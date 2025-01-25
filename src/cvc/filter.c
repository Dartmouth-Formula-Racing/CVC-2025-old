/*
 * filter.c
 *
 * Created on January 7, 2025
 * Andrei Gerashchenko
 */

#include <cvc/can.h>
#include <cvc/data.h>
#include <cvc/filter.h>
#include <math.h>


bool Inverter1_FilteredSpeed_Flag = false;
bool Inverter2_FilteredSpeed_Flag = false;

notchfilter_t left_motor_10;
notchfilter_t left_motor_16;
notchfilter_t right_motor_10;
notchfilter_t right_motor_16;

sample_window window = {
    .data_array = {0}, // Initializes all array elements to 0
    .sample_count = 0
}; // Initialize sample window with all speeds = 0 and sample count = 0

// Function to set up a rolling average filter
int16_t Roll_average(sample_window* window, float new_speed){
    window-> data_array[window->sample_count] = new_speed;
    window->sample_count = (window->sample_count + 1) % WINDOW_SIZE; // Move position indicator on window index

    // Average values in the sample window
    int sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        sum += window->data_array[i];
    }
    // Increase precision of by multiplying and dividing by 1000
    return (int16_t)(RPM_SCALE_FACTOR * (sum / WINDOW_SIZE));
}

void Filter_InitializeNotch(notchfilter_t* filter, uint16_t samplerate, uint16_t center, uint16_t bandwidth) {
    // Solve for coefficients
    filter->omega = 2.0 * PI * center / samplerate;
    filter->alpha = sin(filter->omega) * sinh(log(2.0) / 2.0 * bandwidth * filter->omega / sin(filter->omega));
    filter->a0 = 1.0 + filter->alpha;
    filter->a1 = -2.0 * cos(filter->omega);
    filter->a2 = 1.0 - filter->alpha;
    filter->b0 = 1.0;
    filter->b1 = -2.0 * cos(filter->omega);
    filter->b2 = 1.0;

    // Normalize coefficients
    filter->a1 = filter->a1 / filter->a0;
    filter->a2 = filter->a2 / filter->a0;
    filter->b0 = filter->b0 / filter->a0;
    filter->b1 = filter->b1 / filter->a0;
    filter->b2 = filter->b2 / filter->a0;

    // Initialize state variables
    filter->x1 = 0.0;
    filter->y1 = 0.0;
    filter->x2 = 0.0;
    filter->y2 = 0.0;
}

int16_t Filter_ProcessNotch(notchfilter_t* filter, int16_t sample) {
    // Compute the new output
    float b0 = filter->b0 * sample;
    float b1 = filter->b1 * filter->x1;
    float b2 = filter->b2 * filter->x2;
    float a1 = filter->a1 * filter->y1;
    float a2 = filter->a2 * filter->y2;
    float out = b0 + b1 + b2 - a1 - a2;

    // Update state variables
    filter->x2 = filter->x1;
    filter->x1 = sample;
    filter->y2 = filter->y1;
    filter->y1 = out;

    return (int16_t)out;
}

void Filter_InitializeFilters() {
    // Datalogging shows that most of our oscillations are at 10 and 16 Hz
    Filter_InitializeNotch(&left_motor_10, 333, 10, 6);
    Filter_InitializeNotch(&left_motor_16, 333, 16, 6);
    Filter_InitializeNotch(&right_motor_10, 333, 10, 6);
    Filter_InitializeNotch(&right_motor_16, 333, 16, 6);
}

// Define which filtering is used
#if FILTER_TYPE == 0
    void Filter_ProcessFilterTask() {
    if (Inverter1_HS_Flag) {
        int16_t rpm = (int16_t)CVC_data[INVERTER1_MOTOR_SPEED_HS];
        int16_t filtered = Roll_average(&window, rpm);
        CVC_data[INVERTER1_MOTOR_SPEED_HS_FILTERED] = filtered; // Update CVC_data with left inverter filtered speed (RPM)
        Inverter1_HS_Flag = false;
        Inverter1_FilteredSpeed_Flag = true;
    }
    if (Inverter2_HS_Flag) {
        int16_t rpm = (int16_t)CVC_data[INVERTER2_MOTOR_SPEED_HS];
        int16_t filtered = Roll_average(&window, rpm);
        CVC_data[INVERTER2_MOTOR_SPEED_HS_FILTERED] = filtered; // Update CVC_data with right inverter filtered speed (RPM)
        Inverter2_HS_Flag = false;
        Inverter2_FilteredSpeed_Flag = true;
    }
}
#else if FILTER_TYPE == 1
void Filter_ProcessFilterTask() {
    if (Inverter1_HS_Flag) {
        int16_t rpm = (int16_t)CVC_data[INVERTER1_MOTOR_SPEED_HS];
        int16_t filtered = Filter_ProcessNotch(&left_motor_10, rpm);
        filtered = Filter_ProcessNotch(&left_motor_16, filtered);
        CVC_data[INVERTER1_MOTOR_SPEED_HS_FILTERED] = (uint16_t)filtered; // Update CVC_data with left inverter filtered speed (RPM)
        Inverter1_HS_Flag = false;
        Inverter1_FilteredSpeed_Flag = true;
    }
    if (Inverter2_HS_Flag) {
        int16_t rpm = (int16_t)CVC_data[INVERTER2_MOTOR_SPEED_HS];
        int16_t filtered = Filter_ProcessNotch(&right_motor_10, rpm);
        filtered = Filter_ProcessNotch(&right_motor_16, filtered);
        CVC_data[INVERTER2_MOTOR_SPEED_HS_FILTERED] = (uint16_t)filtered; // Update CVC_data with right inverter filtered speed (RPM)
        Inverter2_HS_Flag = false;
        Inverter2_FilteredSpeed_Flag = true;
    }
}
#endif
