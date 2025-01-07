/*
 * statemachine.h
 *
 * Created on September 19, 2024
 * Andrei Gerashchenko
 */
#ifndef CVC_STATEMACHINE_H
#define CVC_STATEMACHINE_H

#define ENABLE_BUZZER 1
#define BUZZER_TIME 500  // Time in milliseconds for buzzer

// Precharge constants
#define PRECHARGE_TIME 2500      // Time in milliseconds for precharge to take place
#define PRECHARGE_HOLD_TIME 200  // Minimum time in milliseconds for the contactors to be closed before the vehicle can be discharged

#define MIN_PRECHARGE_VOLTAGE 162.0  // Minimum voltage for AIR2 closing (2.7V per cell * 60 cells)
#define MIN_PRECHARGE_PERCENT 0.90   // Threshold for bus voltage to be considered precharged relative to batter voltage (90% of battery voltage)

#define MIN_RTD_THROTTLE 0.05  // Maximum throttle percentage to leave neutral

typedef enum {
    INITIAL,
    VOLTAGE_CHECK,
    WAIT_FOR_PRECHARGE,
    PRECHARGE_STAGE1,
    PRECHARGE_STAGE2,
    PRECHARGE_STAGE3,
    NOT_READY_TO_DRIVE,
    BUZZER,
    READY_TO_DRIVE,
    CHARGING,
} vehicle_state_t;

// Drive state enum
typedef enum {
    NEUTRAL = 0,
    DRIVE,
    REVERSE,
} drive_state_t;

void CVC_StateMachine(void);

#endif  // CVC_STATEMACHINE_H