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

// Satellite Doppler control over the serial port: RAM-only frequency updates
// for the main VFO, several times a second for a whole pass.
//   0x0560 SAT_SET_FREQ   -> 0x0561
//   0x0562 SAT_GET_STATUS -> 0x0563
// Frequencies are dial frequencies in 10 Hz units, all fields little-endian.

#ifndef APP_SATCAT_H
#define APP_SATCAT_H

#include <stdbool.h>
#include <stdint.h>

#define SATCAT_PROTO_VERSION    1

#define SATCAT_APPLY_RX         0x01u
#define SATCAT_APPLY_TX         0x02u

enum {
    SATCAT_OK                = 0x00,
    SATCAT_OK_TX_LATCHED     = 0x01,  // RX applied if asked, TX held until the radio is back in RX
    SATCAT_ERR_BAD_REQUEST   = 0x10,
    SATCAT_ERR_NOT_VFO       = 0x11,  // memory channel on the main VFO
    SATCAT_ERR_CONFIG        = 0x12,  // dual watch or cross band on
    SATCAT_ERR_BUSY          = 0x13,  // scanning, or a modal screen owns the radio
    SATCAT_ERR_RX_RANGE      = 0x14,
    SATCAT_ERR_TX_RANGE      = 0x15,  // TX not allowed there, as RADIO_PrepareTX() decides
};

enum {
    SATCAT_TX_IDLE  = 0,
    SATCAT_TX_KEYED = 1,
    SATCAT_TX_HANG  = 2,  // CW break-in: carrier off, still in TX until the hang time ends
};

// SATCAT_StatusReply_t.Flags
#define SATCAT_STATUS_VFO           0x01u
#define SATCAT_STATUS_CW_CROSS      0x02u
#define SATCAT_STATUS_RIT           0x04u
#define SATCAT_STATUS_XIT           0x08u
#define SATCAT_STATUS_TX_PENDING    0x10u
#define SATCAT_STATUS_DW_XB         0x20u
#define SATCAT_STATUS_POWER_SAVE    0x40u
#define SATCAT_STATUS_TX_ALLOWED    0x80u

typedef struct __attribute__((packed)) {
    uint32_t RxFreq;
    uint32_t TxFreq;
    uint8_t  Flags;     // SATCAT_APPLY_*
    uint8_t  Seq;       // echoed
    uint16_t Reserved;
} SATCAT_SetFreqRequest_t;

typedef struct __attribute__((packed)) {
    uint8_t  Status;
    uint8_t  Seq;
    uint8_t  TxState;
    uint8_t  Flags;     // bit 0: TX latch pending
    uint32_t RxFreq;
    uint32_t TxFreq;    // TX dial in effect (on air when transmitting)
    uint32_t TxPending; // latched TX dial, 0 if none
} SATCAT_SetFreqReply_t;

typedef struct __attribute__((packed)) {
    uint8_t  Proto;
    uint8_t  Seq;
    uint8_t  TxState;
    uint8_t  Modulation;
    uint32_t RxFreq;
    uint32_t TxFreq;
    uint32_t TxPending;
    uint16_t Rssi;      // REG_67 raw, dBm = Rssi / 2 - 160; 0 when not receiving
    int16_t  RitXit;    // 10 Hz units
    uint16_t Step;      // 10 Hz units
    uint8_t  CwTone;    // 10 Hz units
    uint8_t  Flags;     // SATCAT_STATUS_*
    uint32_t Uid[3];    // MCU unique ID, same order as the USB serial number
    uint16_t Battery;   // 10 mV units
    uint16_t Reserved;
    char     Version[16];
} SATCAT_StatusReply_t;

_Static_assert(sizeof(SATCAT_SetFreqRequest_t) == 12, "SAT_SET_FREQ request layout");
_Static_assert(sizeof(SATCAT_SetFreqReply_t) == 16, "SAT_SET_FREQ reply layout");
_Static_assert(sizeof(SATCAT_StatusReply_t) == 56, "SAT_GET_STATUS reply layout");

// modal: called from a modal loop (UART_ServiceCommands), not from APP_Update
void SATCAT_SetFreq(const uint8_t *pData, uint16_t Size, SATCAT_SetFreqReply_t *pReply, bool modal);
void SATCAT_GetStatus(uint8_t Seq, SATCAT_StatusReply_t *pReply);

// FUNCTION_Select(), on every exit from FUNCTION_TRANSMIT
void SATCAT_OnTxEnd(void);

#endif
