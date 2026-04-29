#pragma once

/*
 * camera_control.h
 *
 * Module for remote camera recording control via RC switch and I2C.
 * Sends single-byte commands to Blackmagic 3G SDI Shield over I2C bus.
 *
 * Feature: rc-blackmagic-camera-control
 * Requirements: 2.3, 5.4, 5.5, 6.2
 */

#include <stdint.h>
#include <stdbool.h>
#include "platform.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"
#include "common/time.h"

/* ---------------------------------------------------------------------------
 * Command byte values sent over I2C to SDI Shield
 * ---------------------------------------------------------------------------*/
typedef enum {
    CAMERA_CONTROL_COMMAND_STOP   = 0x00,   // Stop recording (SDI mode=0, Preview)
    CAMERA_CONTROL_COMMAND_RECORD = 0x01,   // Start recording (SDI mode=2, Record)
} cameraControlCommand_e;

/* ---------------------------------------------------------------------------
 * Persistent configuration (stored in INAV EEPROM via PG system)
 * ---------------------------------------------------------------------------*/
typedef struct cameraControlConfig_s {
    uint8_t  i2cAddress;    // I2C address of SDI Shield, default 0x10
    uint8_t  rcChannel;     // RC channel index (0-indexed), default 6 (channel 7)
    uint16_t rcThreshold;   // RC value threshold (normalised CRSF scale 1000-2000), default 1700
    uint16_t debounceMs;    // Debounce time in milliseconds, default 200
} cameraControlConfig_t;

/* ---------------------------------------------------------------------------
 * Internal runtime state (not persisted)
 * ---------------------------------------------------------------------------*/
typedef struct cameraControlState_s {
    bool     lastCommandedState;  // Last successfully sent state (true = Record, false = Stop)
    bool     currentRcState;      // Current RC state after debounce confirmation
    timeMs_t stateChangeTime;     // Timestamp of the last raw RC state change (before debounce)
    bool     pendingStateChange;  // True when a state change is waiting for debounce confirmation
    bool     pendingRcState;      // The pending (unconfirmed) RC state
    uint8_t  i2cRetryCount;       // Current I2C retry attempt counter
    bool     i2cInitialized;      // True when I2C bus has been successfully initialised
} cameraControlState_t;

/* ---------------------------------------------------------------------------
 * INAV persistent-configuration registration
 * ---------------------------------------------------------------------------*/
PG_DECLARE(cameraControlConfig_t, cameraControlConfig);

/* ---------------------------------------------------------------------------
 * Public OSD flag — set/cleared by camera_control, read by OSD module
 * ---------------------------------------------------------------------------*/
extern bool cameraRecordActive;

/* ---------------------------------------------------------------------------
 * Public API
 * ---------------------------------------------------------------------------*/

/**
 * Initialise the camera_control module.
 * Must be called once during FC startup, after I2C bus initialisation and
 * before the scheduler starts.
 */
void cameraControlInit(void);

/**
 * Periodic task called by the INAV scheduler at 50 Hz (every 20 ms).
 * Reads the RC channel value, applies debounce logic, and sends I2C commands
 * when the state changes.
 *
 * @param currentTimeUs  Current system time in microseconds.
 */
void cameraControlTask(timeUs_t currentTimeUs);

/**
 * Send a single-byte command to the SDI Shield over I2C.
 * Implements retry logic: up to 3 retries with 10 ms spacing.
 *
 * @param command  CAMERA_CONTROL_COMMAND_RECORD or CAMERA_CONTROL_COMMAND_STOP.
 * @return         true on success, false if all attempts failed.
 */
bool cameraControlSendCommand(cameraControlCommand_e command);
