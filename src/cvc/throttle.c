/*
* throttle.c
*
* Created on September 20, 2024
* Andrei Gerashchenko
*/
#include <cvc/throttle.h>
#include <cvc/data.h>
#include <cvc/parse.h>
#include <stdbool.h>
#include <main.h>

void Throttle_ProcessThrottle() {
    // CAN_Parse_Inverter_AnalogInputStatus(0); // Parse inverter analog readings

    static uint32_t throttle_invalid_start = 0;
    static bool throttle_invalid_timer = false;
    uint16_t throttle_adc = (uint16_t)CVC_data[CVC_THROTTLE_ADC];
    // volatile float throttle_pulldown = (float)((int32_t)CVC_data[INVERTER1_ANALOG_INPUT_2]) * 0.01;
    bool throttle_valid = false;
    float throttle_position = 0.0;

    // Second throttle input isn't working, so ignoring it for now. Should be fixed ASAP.
    // if (throttle_adc < MIN_THROTTLE_RANGE || throttle_adc > MAX_THROTTLE_RANGE || throttle_pulldown > MAX_THROTTLE_CONTINUITY) {
    if (throttle_adc < MIN_THROTTLE_RANGE || throttle_adc > MAX_THROTTLE_RANGE) {
        if (!throttle_invalid_timer) {
            throttle_invalid_start = HAL_GetTick();
            throttle_invalid_timer = true;
        } else if (HAL_GetTick() - throttle_invalid_start >= THROTTLE_INVALID_TIMER) {
            throttle_valid = false;
        }
    } else {
        throttle_invalid_timer = false;
        throttle_valid = true;
    }

    // Calculate throttle position
    if (throttle_valid) {
        throttle_position = (float)(throttle_adc - THROTTLE_ZERO) / (THROTTLE_FULL - THROTTLE_ZERO);
        if (throttle_position < 0.0) {
            throttle_position = 0.0;
        } else if (throttle_position > 1.0) {
            throttle_position = 1.0;
        }
    } else {
        throttle_position = 0.0;
    }

    CVC_data[CVC_THROTTLE] = (uint16_t)(throttle_position * 1000);
    CVC_data[CVC_THROTTLE_VALID] = throttle_valid;
}