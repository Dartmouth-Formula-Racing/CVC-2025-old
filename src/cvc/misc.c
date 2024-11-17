/*
 * misc.c
 *
 * Created on November 16, 2024
 * Andrei Gerashchenko
 */
#include <cvc/data.h>
#include <cvc/misc.h>
#include <cvc/relay.h>
#include <cvc/parse.h>
#include <cvc/statemachine.h>
#include <main.h>

void CVC_Cooling_Task() {
    CAN_Parse_EMUS_CellTemperatureOverallParameters();
    CAN_Parse_EMUS_CellModuleTemperatureOverallParameters();
    CAN_Parse_Inverter_Temp1(0);
    CAN_Parse_Inverter_Temp1(1);

    int16_t max_cell_temp = CVC_data[BMS_MAX_CELL_TEMP] - 100;
    int16_t max_module_temp = CVC_data[BMS_MAX_CELL_MODULE_TEMP] - 100;
    float inverter1_temp_a = (float)((int16_t)CVC_data[INVERTER1_POWER_MODULE_A_TEMP]) / 10;
    float inverter1_temp_b = (float)((int16_t)CVC_data[INVERTER1_POWER_MODULE_B_TEMP]) / 10;
    float inverter1_temp_c = (float)((int16_t)CVC_data[INVERTER1_POWER_MODULE_C_TEMP]) / 10;
    float inverter2_temp_a = (float)((int16_t)CVC_data[INVERTER2_POWER_MODULE_A_TEMP]) / 10;
    float inverter2_temp_b = (float)((int16_t)CVC_data[INVERTER2_POWER_MODULE_B_TEMP]) / 10;
    float inverter2_temp_c = (float)((int16_t)CVC_data[INVERTER2_POWER_MODULE_C_TEMP]) / 10;

    float max_inverter1_temp = inverter1_temp_a;
    if (inverter1_temp_b > max_inverter1_temp) {
        max_inverter1_temp = inverter1_temp_b;
    }
    if (inverter1_temp_c > max_inverter1_temp) {
        max_inverter1_temp = inverter1_temp_c;
    }

    float max_inverter2_temp = inverter2_temp_a;
    if (inverter2_temp_b > max_inverter2_temp) {
        max_inverter2_temp = inverter2_temp_b;
    }
    if (inverter2_temp_c > max_inverter2_temp) {
        max_inverter2_temp = inverter2_temp_c;
    }

    

    // Cooling pumps should be on any time inverters are on
    if (CVC_data[CVC_STATE] == READY_TO_DRIVE) {
        Relay_Set(Pumps, 1);
    } else {
        Relay_Set(Pumps, 0);
    }
    // Battery fans
    if (max_cell_temp >= BATTERY_FAN_COOLING_TEMP || max_module_temp >= BATTERY_FAN_COOLING_TEMP) {
        Relay_Set(BatteryFans, 1);
    } else if ((max_cell_temp < BATTERY_FAN_COOLING_TEMP - BATTERY_FAN_HYSTERESIS) && (max_module_temp < BATTERY_FAN_COOLING_TEMP - BATTERY_FAN_HYSTERESIS)) {
        Relay_Set(BatteryFans, 0);
    }
    // Radiator fans
    if (max_inverter1_temp >= RADIATOR_FAN_COOLING_TEMP || max_inverter2_temp >= RADIATOR_FAN_COOLING_TEMP) {
        Relay_Set(Fans, 1);
    } else if ((max_inverter1_temp < RADIATOR_FAN_COOLING_TEMP - RADIATOR_FAN_HYSTERESIS) && (max_inverter2_temp < RADIATOR_FAN_COOLING_TEMP - RADIATOR_FAN_HYSTERESIS)) {
        Relay_Set(Fans, 0);
    }

    // Shut down cooling while precharging
    if (CVC_data[CVC_STATE] == PRECHARGE_STAGE1 || CVC_data[CVC_STATE] == PRECHARGE_STAGE2 || CVC_data[CVC_STATE] == PRECHARGE_STAGE3) {
        Relay_Set(BatteryFans, 0);
        Relay_Set(Fans, 0);
    }
}