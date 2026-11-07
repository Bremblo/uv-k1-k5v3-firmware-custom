/* Copyright 2026 KF8CNK
 * https://github.com/Bremblo
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 *     Unless required by applicable law or agreed to in writing, software
 *     distributed under the License is distributed on an "AS IS" BASIS,
 *     WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *     See the License for the specific language governing permissions and
 *     limitations under the License.
 */

// Satellite Doppler control over the serial port (see app/satcat.h).
//
// Everything here is RAM only: no EEPROM or flash writes, no channel save
// request, no VFO reconfigure. Modulation, step, filter, squelch, power,
// offset and codes are left alone. The serial-config countdown is not
// touched either, so updates never block or drop TX.

#include <string.h>

#include "app/chFrScanner.h"
#ifdef ENABLE_CW_MODULATOR
    #include "app/cwrit.h"
#endif
#include "app/satcat.h"
#include "driver/bk4819.h"
#include "driver/bk4819-regs.h"
#include "frequencies.h"
#include "functions.h"
#include "helper/battery.h"
#include "misc.h"
#include "py32f071_ll_utils.h"
#include "radio.h"
#include "settings.h"
#include "version.h"

// TX dial latched while transmitting (10 Hz units, 0 = none) and the VFO it is for
static uint32_t s_tx_pending;
static uint8_t  s_tx_pending_vfo;

static bool IsVfoMode(uint8_t Vfo)
{
    return IS_FREQ_CHANNEL(gEeprom.ScreenChannel[Vfo]);
}

static bool DualWatchOrCrossBand(void)
{
    return gEeprom.DUAL_WATCH != DUAL_WATCH_OFF
        || gEeprom.CROSS_BAND_RX_TX != CROSS_BAND_OFF
        || gRxVfo != gTxVfo;
}

// Same band and same LNA path, so squelch, TX power and the RX filter path
// set up for the current frequency stay valid without being redone
static bool SameBand(uint32_t a, uint32_t b)
{
    return FREQUENCY_GetBand(a) == FREQUENCY_GetBand(b)
        && (a < 28000000) == (b < 28000000);
}

// The test RADIO_PrepareTX() refuses TX with
static bool TxAllowed(const VFO_Info_t *pVfo, uint32_t Frequency)
{
#ifdef ENABLE_FEAT_F4HWN
    return TX_freq_check(Frequency) == 0 || !pVfo->TX_LOCK;
#else
    (void)pVfo;
    return TX_freq_check(Frequency) == 0;
#endif
}

static uint8_t TxState(void)
{
    if (gCurrentFunction != FUNCTION_TRANSMIT)
        return SATCAT_TX_IDLE;
#ifdef ENABLE_CW_MODULATOR
    if (gCW_State == CW_SUSPENDED)
        return SATCAT_TX_HANG;
#endif
    return SATCAT_TX_KEYED;
}

// Retune the receiver in place with the least disturbance: program the
// synthesizer, then give REG_30 the VCO_CALIB 0->1 edge the chip relocks on
// (see BK4819_RX_TurnOn). Every other enable in REG_30 keeps its value, so the
// RX link, DSP and AF DAC stay on. When the receiver is not running (asleep in
// power save) the edge is left to the wake-up's BK4819_RX_TurnOn().
static void Relock(uint32_t Frequency)
{
    BK4819_SetFrequency(Frequency);

    const uint16_t Reg = BK4819_ReadRegister(BK4819_REG_30);
    if ((Reg & BK4819_REG_30_MASK_ENABLE_RX_DSP) == 0)
        return;

    BK4819_WriteRegister(BK4819_REG_30, Reg & (uint16_t)~BK4819_REG_30_MASK_ENABLE_VCO_CALIB);
    BK4819_WriteRegister(BK4819_REG_30, Reg | BK4819_REG_30_MASK_ENABLE_VCO_CALIB);
}

static void ApplyRx(VFO_Info_t *pVfo, uint32_t Frequency)
{
    pVfo->pRX->Frequency = Frequency;

    // while transmitting, RADIO_SetupRegisters() programs it when RX resumes
    if (gCurrentFunction == FUNCTION_TRANSMIT)
        return;

#ifdef ENABLE_CW_MODULATOR
    Frequency = CW_RIT_RxFrequency(pVfo, Frequency);  // CW BFO offset plus RIT
#endif
    Relock(Frequency);
}

// The TX synthesizer is only programmed at key-down (RADIO_SetTxParameters), so
// a latched value just goes into the VFO. A VFO or channel change, or a reload
// to another band, while it was held drops it; the host sees that in status.
static void CommitPendingTx(void)
{
    if (s_tx_pending == 0 || gCurrentFunction == FUNCTION_TRANSMIT)
        return;

    const uint32_t Frequency = s_tx_pending;
    VFO_Info_t    *pVfo      = &gEeprom.VfoInfo[s_tx_pending_vfo];

    s_tx_pending = 0;

    if (!IsVfoMode(s_tx_pending_vfo)
     || !SameBand(Frequency, pVfo->pTX->Frequency)
     || !TxAllowed(pVfo, Frequency))
        return;

    pVfo->pTX->Frequency = Frequency;
    gUpdateDisplay       = true;
}

void SATCAT_OnTxEnd(void)
{
    CommitPendingTx();
}

void SATCAT_SetFreq(const uint8_t *pData, uint16_t Size, SATCAT_SetFreqReply_t *pReply, bool modal)
{
    SATCAT_SetFreqRequest_t Req;
    VFO_Info_t             *pVfo   = gTxVfo;
    uint8_t                 Status = SATCAT_OK;

    memset(&Req, 0, sizeof(Req));
    memcpy(&Req, pData, Size < sizeof(Req) ? Size : sizeof(Req));

    CommitPendingTx();  // a TX end may have passed without the hook (belt and braces)

    if (Size != sizeof(Req) || Req.Reserved != 0
     || (Req.Flags & (SATCAT_APPLY_RX | SATCAT_APPLY_TX)) == 0
     || (Req.Flags & ~(SATCAT_APPLY_RX | SATCAT_APPLY_TX)) != 0)
        Status = SATCAT_ERR_BAD_REQUEST;
    else if (modal || gScanStateDir != SCAN_OFF)
        Status = SATCAT_ERR_BUSY;
    else if (!IsVfoMode(gEeprom.TX_VFO))
        Status = SATCAT_ERR_NOT_VFO;
    else if (DualWatchOrCrossBand())
        Status = SATCAT_ERR_CONFIG;
    else if ((Req.Flags & SATCAT_APPLY_RX)
          && (RX_freq_check(Req.RxFreq) != 0 || !SameBand(Req.RxFreq, pVfo->pRX->Frequency)))
        Status = SATCAT_ERR_RX_RANGE;
    else if ((Req.Flags & SATCAT_APPLY_TX)
          && (!TxAllowed(pVfo, Req.TxFreq) || !SameBand(Req.TxFreq, pVfo->pTX->Frequency)))
        Status = SATCAT_ERR_TX_RANGE;
    else
    {
        if (Req.Flags & SATCAT_APPLY_RX)
            ApplyRx(pVfo, Req.RxFreq);

        if (Req.Flags & SATCAT_APPLY_TX)
        {
            if (gCurrentFunction == FUNCTION_TRANSMIT)
            {   // never retune during key-down: hold it for the next return to RX
                s_tx_pending     = Req.TxFreq;
                s_tx_pending_vfo = gEeprom.TX_VFO;
                Status           = SATCAT_OK_TX_LATCHED;
            }
            else
            {
                pVfo->pTX->Frequency = Req.TxFreq;
                s_tx_pending         = 0;
            }
        }

        // no redraw while transmitting: it would only delay the keyer, and TX end redraws
        if (gCurrentFunction != FUNCTION_TRANSMIT)
            gUpdateDisplay = true;
    }

    pReply->Status    = Status;
    pReply->Seq       = Req.Seq;
    pReply->TxState   = TxState();
    pReply->Flags     = s_tx_pending != 0;
    pReply->RxFreq    = pVfo->pRX->Frequency;
    pReply->TxFreq    = pVfo->pTX->Frequency;
    pReply->TxPending = s_tx_pending;
}

void SATCAT_GetStatus(uint8_t Seq, SATCAT_StatusReply_t *pReply)
{
    const VFO_Info_t *pVfo  = gTxVfo;
    uint8_t           Flags = 0;

    CommitPendingTx();

    memset(pReply, 0, sizeof(*pReply));

    pReply->Proto      = SATCAT_PROTO_VERSION;
    pReply->Seq        = Seq;
    pReply->TxState    = TxState();
    pReply->Modulation = pVfo->Modulation;
    pReply->RxFreq     = pVfo->pRX->Frequency;
    pReply->TxFreq     = pVfo->pTX->Frequency;
    pReply->TxPending  = s_tx_pending;
    pReply->Step       = pVfo->StepFrequency;

    if (gCurrentFunction != FUNCTION_TRANSMIT && gCurrentFunction != FUNCTION_POWER_SAVE)
        pReply->Rssi = BK4819_GetRSSI();

#ifdef ENABLE_CW_MODULATOR
    pReply->RitXit = CW_RIT_Offset();
    pReply->CwTone = gEeprom.CW_TONE_FREQUENCY;
    if (gCW_CrossMode)
        Flags |= SATCAT_STATUS_CW_CROSS;
    if (CW_RIT_RitOn())
        Flags |= SATCAT_STATUS_RIT;
    if (CW_RIT_XitOn())
        Flags |= SATCAT_STATUS_XIT;
#endif

    if (IsVfoMode(gEeprom.TX_VFO))
        Flags |= SATCAT_STATUS_VFO;
    if (s_tx_pending != 0)
        Flags |= SATCAT_STATUS_TX_PENDING;
    if (DualWatchOrCrossBand())
        Flags |= SATCAT_STATUS_DW_XB;
    if (gCurrentFunction == FUNCTION_POWER_SAVE)
        Flags |= SATCAT_STATUS_POWER_SAVE;
    if (TxAllowed(pVfo, pVfo->pTX->Frequency))
        Flags |= SATCAT_STATUS_TX_ALLOWED;
    pReply->Flags = Flags;

    pReply->Uid[0]  = LL_GetUID_Word0();
    pReply->Uid[1]  = LL_GetUID_Word1();
    pReply->Uid[2]  = LL_GetUID_Word2();
    pReply->Battery = gBatteryVoltageAverage;
    strncpy(pReply->Version, Version, sizeof(pReply->Version));
}
