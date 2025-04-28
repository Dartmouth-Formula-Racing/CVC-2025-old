/*
Author: Sasha Ries
Date: 2/10/25
File: traction_control.c
Decription: Module to calculate slip ratio and a dynamic torque scaling factor
*/

#include <cvc/data.h>
#include <cvc/parse.h>
#include <cvc/traction_control.h>


// Define the size of the rolling window
#define WINDOW_SIZE 50

// Create a struct for the rolling window
typedef struct {
    uint16_t values[WINDOW_SIZE];  // Array to store the values
    uint16_t current_index;         // Current index to insert the next value
    uint16_t count;                 // Number of values currently in the window
} RollingWindow;

RollingWindow speed_window = {0}; // Struct to hold speeds for debugging purposes

// Update the rolling window with a new value
void update_value(uint16_t new_value) {
    // Store the new value at the current index
    speed_window.values[speed_window.current_index] = new_value;
    
    // Increment the current index, wrapping around if necessary
    speed_window.current_index = (speed_window.current_index + 1) % WINDOW_SIZE;
    
    // Update the count of values in the window (up to WINDOW_SIZE)
    if (speed_window.count < WINDOW_SIZE) {
        speed_window.count++;
    }
}


float Traction_Control_Update(TractionControl_t *tc, float current_slip) {
    // Calculate error (positive error means slip is too high)
    float error = current_slip - tc->optimal_slip;

    // If slip is below optimal, no need to reduce torque
    if (error <= 0.0f) {
        return 1.0f;  // Return full torque (no reduction)
    }

    // Simple proportional control - only using P term
    // CHARACTERIZATION NEEDED: kp value must be between 1 and 0:
    float p_term = tc->kp * error;

    // Convert controller output to a torque scaling factor
    // Higher control output = more reduction
    float torque_factor = 1.0f - p_term;

    // - output_limit_min: Maximum torque reduction (e.g., 0.2 = can cut up to
    // 80% of torque)
    // - output_limit_max: Always 1.0 for full torque when no intervention is
    // needed
    if (torque_factor < tc->output_limit_min) {
        torque_factor = tc->output_limit_min;
    } else if (torque_factor > tc->output_limit_max) {
        torque_factor = tc->output_limit_max;
    }

    return torque_factor;
}

void Traction_Control_Reset(TractionControl_t *tc) {
    tc->error_previous = 0.0f;
    tc->error_integral = 0.0f;
}

void Traction_Calculate_SlipRatio() {
    CAN_Parse_SensorBoard_FilteredSpeeds();
    CAN_Parse_Inverter_HighSpeedParameters(0);
    CAN_Parse_Inverter_HighSpeedParameters(1);
    // Get the front and rear wheel speeds for both sides
    uint16_t front_left_speed = (uint16_t)(CVC_data[SENSOR_LEFT_WHEELSPEED]);
    uint16_t front_right_speed = (uint16_t)(CVC_data[SENSOR_RIGHT_WHEELSPEED]);
    uint16_t rear_left_speed = (uint16_t)CVC_data[INVERTER1_MOTOR_SPEED_HS];
    uint16_t rear_right_speed = (uint16_t)CVC_data[INVERTER2_MOTOR_SPEED_HS];

    //update_value(front_right_speed);


    // Ensure all speeds are positive
    // front_left_speed = (front_left_speed < 0) ? -front_left_speed : front_left_speed;
    // front_right_speed = (front_right_speed < 0) ? -front_right_speed : front_right_speed;
    // rear_left_speed = (rear_left_speed < 0) ? -rear_left_speed : rear_left_speed;
    // rear_right_speed = (rear_right_speed < 0) ? -rear_right_speed : rear_right_speed;

    if (rear_left_speed <= MIN_REAR_SPEED || rear_right_speed <= MIN_REAR_SPEED) {
        CVC_data[CVC_SLIP_RATIO_LEFT] = 0;
        CVC_data[CVC_SLIP_RATIO_RIGHT] = 0;
    } else {
        CVC_data[CVC_SLIP_RATIO_LEFT] = (uint64_t)(((float)(rear_left_speed - front_left_speed) / (float)rear_left_speed) * RPM_SCALE_FACTOR);
        CVC_data[CVC_SLIP_RATIO_RIGHT] = (uint64_t)(((float)(rear_right_speed - front_right_speed) / (float)rear_right_speed) * RPM_SCALE_FACTOR);
    }

    //update_value((uint16_t)(((float)(rear_left_speed - front_left_speed) / (float)rear_left_speed) * RPM_SCALE_FACTOR));

}