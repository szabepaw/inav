/*
 * camera_control.c
 *
 * Module for remote camera recording control via RC switch and I2C.
 * Sends single-byte commands to Blackmagic 3G SDI Shield over I2C bus.
 *
 * Feature: rc-blackmagic-camera-control
 * Requirements: 2.3, 5.4, 5.5, 6.2
 */

#include <string.h>

#include "camera_control.h"
#include "common/time.h"
#include "drivers/bus_i2c.h"
#include "drivers/time.h"
#include "fc/runtime_config.h"
#include "rx/rx.h"
#include "flight/failsafe.h"

/* ---------------------------------------------------------------------------
 * I2C retry constants
 * ---------------------------------------------------------------------------*/
#define CAMERA_CONTROL_I2C_MAX_RETRIES    3
#define CAMERA_CONTROL_I2C_RETRY_DELAY_MS 10

/* ---------------------------------------------------------------------------
 * Persistent configuration — registered in INAV EEPROM via PG system.
 *
 * PG_CAMERA_CONTROL_CONFIG = 1045  (defined in src/main/config/parameter_group_ids.h)
 * ---------------------------------------------------------------------------*/
PG_REGISTER_WITH_RESET_TEMPLATE(cameraControlConfig_t, cameraControlConfig,
                                PG_CAMERA_CONTROL_CONFIG, 0);

pg_resetdata_decl const cameraControlConfig_t pgResetTemplate_cameraControlConfig = {
    .i2cAddress  = 0x10,   // Default I2C address of SDI Shield
    .rcChannel   = 6,      // Default RC channel index (0-indexed → channel 7)
    .rcThreshold = 1700,   // Default threshold on normalised CRSF scale 1000-2000
    .debounceMs  = 200,    // Default debounce time in milliseconds
};

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
    // rcInput[] jest tablicą globalną w INAV, indeksowaną 0-based
    const uint16_t rcValue = rcInput[config->rcChannel];

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

    const cameraControlConfig_t *config = cameraControlConfig();
    uint8_t cmdByte = (uint8_t)command;
    bool success = false;

    state.i2cRetryCount = 0;

    for (int attempt = 0; attempt <= CAMERA_CONTROL_I2C_MAX_RETRIES; attempt++) {
        if (attempt > 0) {
            // Odczekaj przed kolejną próbą
            delay(CAMERA_CONTROL_I2C_RETRY_DELAY_MS);
        }

        // Wyślij pojedynczy bajt polecenia przez I2C
        // i2cWriteBuffer(device, addr, len, data)
        // Używamy I2CDEV_1 — domyślny I2C dla zewnętrznych urządzeń na MAMBA F722
        if (i2cWriteBuffer(I2CDEV_1, config->i2cAddress, 1, &cmdByte)) {
            success = true;
            break;
        }

        state.i2cRetryCount++;
    }

    if (success) {
        // Zaktualizuj stan i flagę OSD
        state.lastCommandedState = (command == CAMERA_CONTROL_COMMAND_RECORD);
        cameraControlUpdateOSD(state.lastCommandedState);
    }
    // Przy 3 nieudanych próbach błąd jest rejestrowany przez warstwę I2C INAV.
    // Rozszerzone logowanie diagnostyczne może być dodane tutaj gdy dostępne
    // będzie API loggera (np. LOG_E(CAMERA_CONTROL, ...)).

    return success;
}
