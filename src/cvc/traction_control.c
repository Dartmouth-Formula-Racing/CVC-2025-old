/*
Author: Sasha Ries
Date: 2/10/25
File: traction_control.c
Decription: Module to calculate slip ratio and a dynamic torque scaling factor
*/

#include <cvc/data.h>
#include "traction_control.h"


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
    
    // - output_limit_min: Maximum torque reduction (e.g., 0.2 = can cut up to 80% of torque)
    // - output_limit_max: Always 1.0 for full torque when no intervention is needed
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


void calculate_slip_ratio(){
    // Get the front and rear wheel speeds for both sides
    float front_left_speed = (float)((uint32_t)(CVC_data[SENSOR_LEFT_WHEELSPEED]));
    float front_right_speed = (float)((uint32_t)(CVC_data[SENSOR_RIGHT_WHEELSPEED]));
    float rear_left_speed = (float)((uint32_t)CVC_data[INVERTER1_MOTOR_SPEED_HS_FILTERED]);
    float rear_right_speed = (float)((uint32_t)CVC_data[INVERTER2_MOTOR_SPEED_HS_FILTERED]);
    // float rear_left_speed = (float)((uint32_t)(CVC_data[INVERTER1_MOTOR_SPEED_HS]));
    // float rear_right_speed = (float)((uint32_t)(CVC_data[INVERTER2_MOTOR_SPEED_HS]));

    // Update CVC_data with slip ratios of both sides and multiply by RPM_SCAlE_FACTOR

    if (rear_left_speed <= MIN_REAR_SPEED || rear_right_speed <= MIN_REAR_SPEED){
        CVC_data[CVC_SLIP_RATIO_LEFT] = 0;
        CVC_data[CVC_SLIP_RATIO_RIGHT] = 0;
    }else{
        CVC_data[CVC_SLIP_RATIO_LEFT] = (uint16_t)(((rear_left_speed - front_left_speed) / rear_left_speed)*RPM_SCALE_FACTOR);
        CVC_data[CVC_SLIP_RATIO_RIGHT] = (uint16_t)(((rear_right_speed - front_right_speed) / rear_right_speed)*RPM_SCALE_FACTOR);
    }
}