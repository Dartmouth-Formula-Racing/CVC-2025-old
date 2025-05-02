/*
 * statemachine.c
 *
 * Created on September 19, 2024
 * Andrei Gerashchenko
 */

#include <cvc/data.h>
#include <cvc/parse.h>
#include <cvc/relay.h>
#include <cvc/statemachine.h>
#include <cvc/torque.h>
#include <main.h>
#include <stdbool.h>

void CVC_StateMachine() {
    CAN_Parse_EMUS_OverallParameters();                // BMS overall parameters
    CAN_Parse_EMUS_BatteryVoltageOverallParameters();  // HV battery voltage
    CAN_Parse_Inverter_VoltageParameters(0);           // Inverter 1 voltage - Fast message
    CAN_Parse_Inverter_VoltageParameters(1);           // Inverter 2 voltage - Fast message
    CAN_Parse_Dashboard();                             // Dashboard state

    volatile vehicle_state_t state = CVC_data[CVC_STATE];
    volatile drive_state_t drive_mode = NEUTRAL;
    volatile drive_state_t requested_drive_mode = CVC_data[DASH_REQUESTED_STATE];
    volatile bool air2 = false;
    volatile bool buzzer = false;

    volatile float HV_voltage = CVC_data[BMS_TOTAL_VOLTAGE] * 0.01;
    volatile float Inverter1_voltage = (float)((int16_t)CVC_data[INVERTER1_DC_BUS_VOLTAGE]) * 0.1;  // Fast message
    volatile float Inverter2_voltage = (float)((int16_t)CVC_data[INVERTER1_DC_BUS_VOLTAGE]) * 0.1;  // Fast message
    volatile float throttle = (float)((uint16_t)CVC_data[CVC_THROTTLE]) * 0.01;
    volatile bool charging = CVC_data[BMS_CHARGING_STATE] > 0;

    // Internal timers
    static volatile uint32_t precharge_start_time = 0;
    static volatile uint32_t contactor_close_time = 0;
    static volatile uint32_t buzzer_start_time = 0;

    // Drive lockout
    static volatile bool drive_lockout = true;

    if (requested_drive_mode == NEUTRAL) {
        drive_lockout = false;
    }

    // Drive mode always becomes neutral unless otherwise specified
    // Second AIR always open unless otherwise specified
    switch (state) {
        case INITIAL:
            state = VOLTAGE_CHECK;
            break;
        case VOLTAGE_CHECK:
            // TODO: Check voltages on startup
            state = WAIT_FOR_PRECHARGE;
            break;
        case WAIT_FOR_PRECHARGE:  // Waiting for precharge button press
            // Precharge begins when AIR 1 is closed and precharge button is pressed
            if (CVC_data[CVC_PRECHARGE_BTN] == 1 && CVC_data[CVC_AIR_1_STATE] && HV_voltage >= MIN_PRECHARGE_VOLTAGE) {
                state = PRECHARGE_STAGE1;
                precharge_start_time = HAL_GetTick();
            } else if (HV_voltage >= MIN_PRECHARGE_VOLTAGE && Inverter1_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT && Inverter2_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT) {
                state = PRECHARGE_STAGE3;
                precharge_start_time = HAL_GetTick();
            } else {
                state = WAIT_FOR_PRECHARGE;
            }
            break;
        case PRECHARGE_STAGE1:  // Waiting for precharge timer to expire
            if (HAL_GetTick() - precharge_start_time >= PRECHARGE_TIME) {
                state = PRECHARGE_STAGE2;
            } else {
                state = PRECHARGE_STAGE1;
            }
            break;
        case PRECHARGE_STAGE2:  // Enough time has passed for precharge, check if voltage is high enough
            if (Inverter1_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT && Inverter2_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT) {
                state = PRECHARGE_STAGE3;
                air2 = true;
                contactor_close_time = HAL_GetTick();
            } else {
                state = WAIT_FOR_PRECHARGE;
                break;
            }
            break;
        case PRECHARGE_STAGE3:  // Contactor has closed, make sure that nothing has opened it
            if (HAL_GetTick() - contactor_close_time >= PRECHARGE_HOLD_TIME) {
                if (Inverter1_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT && Inverter2_voltage >= HV_voltage * MIN_PRECHARGE_PERCENT) {
                    state = NOT_READY_TO_DRIVE;
                    air2 = true;
                } else {
                    state = WAIT_FOR_PRECHARGE;
                    break;
                }
            } else {
                state = PRECHARGE_STAGE3;
                air2 = true;
            }
            break;
        case NOT_READY_TO_DRIVE:  // Both contactors are closed, inverters are disabled
            // Check if second contactor should be open
            if (Inverter1_voltage < HV_voltage * MIN_PRECHARGE_PERCENT || Inverter2_voltage < HV_voltage * MIN_PRECHARGE_PERCENT) {
                state = WAIT_FOR_PRECHARGE;
                break;
            } else {
                state = NOT_READY_TO_DRIVE;
                air2 = true;
            }

            if (charging) {
                state = CHARGING;
                break;
            }

            if (!drive_lockout && (requested_drive_mode == DRIVE || requested_drive_mode == REVERSE)) {
                // Check if throttle is valid and not pressed
                if (CVC_data[CVC_THROTTLE_VALID] && throttle <= MIN_RTD_THROTTLE) {
                    state = BUZZER;  // Start buzzer
                    buzzer_start_time = HAL_GetTick();
                    drive_lockout = true;
                    // Use time during buzzer to clear inverter faults
                    if (!Inverter1_Clear_Flag) {
                        Inverter1_Clear_Flag = true;
                    }
                    if (!Inverter2_Clear_Flag) {
                        Inverter2_Clear_Flag = true;
                    }
                } else {
                    state = NOT_READY_TO_DRIVE;
                    drive_lockout = true;
                }
            } else {
                state = NOT_READY_TO_DRIVE;
            }
            break;
        case BUZZER:  // Buzzer needs to be on for some time before ready to drive
            // Check if second contactor should be open
            if (Inverter1_voltage < HV_voltage * MIN_PRECHARGE_PERCENT || Inverter2_voltage < HV_voltage * MIN_PRECHARGE_PERCENT) {
                state = WAIT_FOR_PRECHARGE;
                break;
            } else {
                state = BUZZER;
                air2 = true;
            }

            // Check if vehicle is charging
            if (charging) {
                state = CHARGING;
                buzzer = false;
                break;
            }

            if (HAL_GetTick() - buzzer_start_time >= BUZZER_TIME) {
                state = READY_TO_DRIVE;
                buzzer = false;
            } else {
                state = BUZZER;
                buzzer = true;
            }
            break;
        case CHARGING:
            // Check if second contactor should be open
            if (Inverter1_voltage < HV_voltage * MIN_PRECHARGE_PERCENT || Inverter2_voltage < HV_voltage * MIN_PRECHARGE_PERCENT) {
                state = WAIT_FOR_PRECHARGE;
                break;
            } else {
                state = CHARGING;
                air2 = true;
            }
            if (charging) {
                state = CHARGING;
                drive_lockout = true;
            } else {
                state = NOT_READY_TO_DRIVE;
            }
        case READY_TO_DRIVE:
            // Check if second contactor should be open
            if (Inverter1_voltage < MIN_PRECHARGE_VOLTAGE || Inverter2_voltage < MIN_PRECHARGE_VOLTAGE) {
                state = WAIT_FOR_PRECHARGE;
                break;
            } else {
                state = READY_TO_DRIVE;
                air2 = true;
            }

            // Check if vehicle is charging
            if (charging) {
                state = CHARGING;
                drive_lockout = true;
                break;
            }

            // Check if throttle is invalid
            if (!CVC_data[CVC_THROTTLE_VALID]) {
                state = WAIT_FOR_PRECHARGE;
                break;
            }

            if (requested_drive_mode == NEUTRAL) {
                state = NOT_READY_TO_DRIVE;
            } else if (requested_drive_mode == DRIVE && CVC_data[CVC_DRIVE_MODE] != REVERSE) {
                drive_mode = DRIVE;
            } else if (requested_drive_mode == REVERSE && CVC_data[CVC_DRIVE_MODE] != DRIVE) {
                drive_mode = REVERSE;
            } else {
                state = NOT_READY_TO_DRIVE;
            }
            break;
        default:
            state = WAIT_FOR_PRECHARGE;
            break;
    }

    CVC_data[CVC_STATE] = state;
    CVC_data[CVC_DRIVE_MODE] = drive_mode;
    Relay_Set(AIR2, air2);
    if (ENABLE_BUZZER) {
        Relay_Set(Buzzer, buzzer);
    }
}