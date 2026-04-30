/*
 * camera_control.c
 *
 * Module for remote camera recording control via RC switch and I2C.
 * Communicates DIRECTLY with Blackmagic 3G SDI Shield over I2C bus
 * using the shield's register protocol — no Arduino intermediary needed.
 *
 * SDI Shield I2C register protocol (address 0x6E by default):
 *   Each I2C write: [reg_addr_low] [reg_addr_high] [data...]
 *
 *   0x1000 (CONTROL)  — bit 0: camera control override enable
 *   0x2001 (OCLENGTH) — outgoing camera control packet length
 *   0x2100 (OCDATA)   — outgoing camera control packet data (up to 255 bytes)
 *
 * SDI Camera Control packet (Blackmagic protocol v1.6.2):
 *   [dest=0xFF][len=5][cmd=0][0x00][cat=10][par=1][type=1][op=0][mode][pad][pad][pad]
 *   mode 0 = Preview/Stop, mode 2 = Record
 *
 * Feature: rc-blackmagic-camera-control
 * Requirements: 2.3, 5.4, 5.5, 6.2
 */

#include <string.h>

#include "camera_control.h"
#include "common/time.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "rx/rx.h"
#include "flight/failsafe.h"

#ifndef SITL_BUILD
#include "drivers/bus_i2c.h"
#endif

/* ---------------------------------------------------------------------------
 * I2C retry constants
 * ---------------------------------------------------------------------------*/
#define CAMERA_CONTROL_I2C_MAX_RETRIES    3
#define CAMERA_CONTROL_I2C_RETRY_DELAY_MS 10

/* ---------------------------------------------------------------------------
 * SDI Shield register addresses (from BMDSDIControlShieldRegisters.h)
 * ---------------------------------------------------------------------------*/
#define SDI_REG_CONTROL   0x1000  // Control register (bit 0 = camera override)
#define SDI_REG_OCLENGTH  0x2001  // Outgoing camera control packet length
#define SDI_REG_OCDATA    0x2100  // Outgoing camera control packet data

#define SDI_CONTROL_COVERIDE_MASK  0x01  // Camera control override bit

/* ---------------------------------------------------------------------------
 * SDI Camera Control packet constants
 * Blackmagic SDI Camera Control Protocol v1.6.2
 * Category 10 (Media), Parameter 1 (Transport mode)
 * ---------------------------------------------------------------------------*/
#define SDI_CAMERA_BROADCAST  0xFF  // Destination: all cameras
#define SDI_PAYLOAD_LENGTH    5     // Bytes 4-8 = 5 bytes of payload
#define SDI_CMD_CHANGE_CONFIG 0x00  // Command: change configuration
#define SDI_CATEGORY_MEDIA    10    // Category: Media
#define SDI_PARAM_TRANSPORT   1     // Parameter: Transport mode
#define SDI_TYPE_INT8         1     // Data type: signed byte
#define SDI_OP_ASSIGN         0     // Operation: assign value
#define SDI_TRANSPORT_PREVIEW 0     // mode=0: Preview/Stop
#define SDI_TRANSPORT_RECORD  2     // mode=2: Record
#define SDI_PACKET_LENGTH     12    // Total packet length (padded to 32-bit)

/* ---------------------------------------------------------------------------
 * Persistent configuration — registered in INAV EEPROM via PG system.
 *
 * PG_CAMERA_CONTROL_CONFIG = 1045  (defined in src/main/config/parameter_group_ids.h)
 *
 * i2cAddress: SDI Shield I2C address (default 0x6E — set by jumpers on shield)
 * ---------------------------------------------------------------------------*/
PG_REGISTER_WITH_RESET_TEMPLATE(cameraControlConfig_t, cameraControlConfig,
                                PG_CAMERA_CONTROL_CONFIG, 0);

PG_RESET_TEMPLATE(cameraControlConfig_t, cameraControlConfig,
    .i2cAddress  = 0x6E,   // Default SDI Shield I2C address (jumper-set)
    .rcChannel   = 6,
    .rcThreshold = 1700,
    .debounceMs  = 200,
);

/* ---------------------------------------------------------------------------
 * Public OSD flag — read by the OSD module to render the REC indicator.
 * ---------------------------------------------------------------------------*/
bool cameraRecordActive = false;

/* ---------------------------------------------------------------------------
 * Internal runtime state
 * ---------------------------------------------------------------------------*/
static cameraControlState_t state;

/* ---------------------------------------------------------------------------
 * Private helpers
 * ---------------------------------------------------------------------------*/

/**
 * Update the OSD recording flag.
 * Called after every successful I2C command transmission.
 *
 * @param isRecording  true when recording is active, false otherwise.
 */
static void cameraControlUpdateOSD(bool isRecording)
{
    cameraRecordActive = isRecording;
}

/* ---------------------------------------------------------------------------
 * Public API — implementations (logic filled in subsequent tasks)
 * ---------------------------------------------------------------------------*/

void cameraControlInit(void)
{
    // Zainicjalizuj stan wewnętrzny na bezpieczne wartości domyślne.
    memset(&state, 0, sizeof(state));
    state.lastCommandedState = false;
    state.currentRcState     = false;
    state.stateChangeTime    = 0;
    state.pendingStateChange = false;
    state.pendingRcState     = false;
    state.i2cRetryCount      = 0;
    state.i2cInitialized     = false;

    // Wyczyść flagę OSD — nagrywanie nieaktywne po starcie.
    cameraRecordActive = false;

    // I2C jest inicjalizowane globalnie przez INAV (dla magnetometru, barometru itp.)
    // w sekwencji startowej FC przed wywołaniem cameraControlInit().
    // Moduł camera_control zakłada, że magistrala I2C jest już dostępna
    // i tylko oznacza ją jako gotową do użycia.
    state.i2cInitialized = true;
}

void cameraControlTask(timeUs_t currentTimeUs)
{
    if (!state.i2cInitialized) {
        return;
    }

    // Obsługa failsafe RC — utrata sygnału RC
    if (!rxIsReceivingSignal()) {
        // Sygnał RC utracony — zatrzymaj nagrywanie jeśli było aktywne
        if (state.lastCommandedState) {
            cameraControlSendCommand(CAMERA_CONTROL_COMMAND_STOP);
        }
        // Wyczyść stan debounce
        state.pendingStateChange = false;
        state.currentRcState     = false;
        return;
    }

    const cameraControlConfig_t *config = cameraControlConfig();
    const timeMs_t nowMs = currentTimeUs / 1000;

    // Odczytaj wartość kanału RC (INAV normalizuje CRSF do 1000-2000 wewnętrznie)
    // W INAV 8.x używamy rxGetChannelValue() zamiast rcInput[]
    const uint16_t rcValue = (uint16_t)rxGetChannelValue(config->rcChannel);

    // Wyznacz nowy stan RC na podstawie progu
    const bool newRcState = (rcValue > config->rcThreshold);

    // Logika debounce
    if (newRcState != state.currentRcState) {
        // Stan RC zmienił się — sprawdź czy to nowa zmiana czy kontynuacja oczekiwania
        if (!state.pendingStateChange || (state.pendingRcState != newRcState)) {
            // Nowa zmiana stanu — zacznij odliczać debounce
            state.pendingStateChange = true;
            state.pendingRcState     = newRcState;
            state.stateChangeTime    = nowMs;
        } else {
            // Oczekujemy na potwierdzenie debounce — sprawdź czy minął czas
            if ((nowMs - state.stateChangeTime) >= config->debounceMs) {
                // Debounce potwierdzony — zmień stan
                state.currentRcState     = newRcState;
                state.pendingStateChange = false;

                // Wyślij polecenie I2C tylko jeśli stan się faktycznie zmienił
                // względem ostatnio wysłanego polecenia
                if (state.currentRcState != state.lastCommandedState) {
                    const cameraControlCommand_e cmd = state.currentRcState
                        ? CAMERA_CONTROL_COMMAND_RECORD
                        : CAMERA_CONTROL_COMMAND_STOP;
                    cameraControlSendCommand(cmd);
                }
            }
        }
    } else {
        // Stan RC nie zmienił się — anuluj oczekujące zmiany
        state.pendingStateChange = false;
    }
}

bool cameraControlSendCommand(cameraControlCommand_e command)
{
    if (!state.i2cInitialized) {
        return false;
    }

    bool success = false;
    state.i2cRetryCount = 0;

#ifndef SITL_BUILD
    const cameraControlConfig_t *config = cameraControlConfig();

    // Build 12-byte SDI Camera Control packet
    // Blackmagic SDI Camera Control Protocol v1.6.2
    // Category 10 (Media), Parameter 1 (Transport mode)
    const int8_t transportMode = (command == CAMERA_CONTROL_COMMAND_RECORD)
        ? SDI_TRANSPORT_RECORD
        : SDI_TRANSPORT_PREVIEW;

    uint8_t packet[SDI_PACKET_LENGTH] = {
        SDI_CAMERA_BROADCAST,   // [0] Destination: broadcast
        SDI_PAYLOAD_LENGTH,     // [1] Payload length
        SDI_CMD_CHANGE_CONFIG,  // [2] Command: change configuration
        0x00,                   // [3] Reserved
        SDI_CATEGORY_MEDIA,     // [4] Category: 10 (Media)
        SDI_PARAM_TRANSPORT,    // [5] Parameter: 1 (Transport mode)
        SDI_TYPE_INT8,          // [6] Data type: 1 (signed byte)
        SDI_OP_ASSIGN,          // [7] Operation: 0 (assign)
        (uint8_t)transportMode, // [8] Data[0]: transport mode
        0x00,                   // [9]  Padding
        0x00,                   // [10] Padding
        0x00,                   // [11] Padding
    };

    for (int attempt = 0; attempt <= CAMERA_CONTROL_I2C_MAX_RETRIES; attempt++) {
        if (attempt > 0) {
            delay(CAMERA_CONTROL_I2C_RETRY_DELAY_MS);
        }

        // Step 1: Enable camera control override
        // Write 0x01 to register 0x1000 (CONTROL)
        // I2C format: [reg_low=0x00] [reg_high=0x10] [value=0x01]
        uint8_t controlVal = SDI_CONTROL_COVERIDE_MASK;
        if (!i2cWriteBuffer(I2CDEV_1, config->i2cAddress, 0x00, 2,
                            (uint8_t[]){0x10, controlVal}, false)) {
            state.i2cRetryCount++;
            continue;
        }

        // Step 2: Write packet length to register 0x2001 (OCLENGTH)
        // I2C format: [reg_low=0x01] [reg_high=0x20] [length=12]
        uint8_t lenData[3] = {0x01, 0x20, SDI_PACKET_LENGTH};
        if (!i2cWriteBuffer(I2CDEV_1, config->i2cAddress, 0xFF, 3,
                            lenData, true)) {
            state.i2cRetryCount++;
            continue;
        }

        // Step 3: Write packet data to register 0x2100 (OCDATA)
        // I2C format: [reg_low=0x00] [reg_high=0x21] [12 bytes of packet]
        uint8_t dataMsg[2 + SDI_PACKET_LENGTH];
        dataMsg[0] = 0x00;  // reg_low
        dataMsg[1] = 0x21;  // reg_high
        memcpy(&dataMsg[2], packet, SDI_PACKET_LENGTH);
        if (!i2cWriteBuffer(I2CDEV_1, config->i2cAddress, 0xFF,
                            2 + SDI_PACKET_LENGTH, dataMsg, true)) {
            state.i2cRetryCount++;
            continue;
        }

        success = true;
        break;
    }
#else
    // SITL: no hardware I2C — simulate success
    (void)command;
    success = true;
#endif

    if (success) {
        state.lastCommandedState = (command == CAMERA_CONTROL_COMMAND_RECORD);
        cameraControlUpdateOSD(state.lastCommandedState);
    }

    return success;
}
