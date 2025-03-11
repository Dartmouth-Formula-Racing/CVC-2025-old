/*
Author: Sasha Ries
Date: 2/10/25
File: traction_control.c
Decription: Module to calculate slip ratio and a dynamic torque scaling factor
*/

#include <cvc/data.h>
#include "traction_control.h"

void calculate_slip_ratio(){
    // Get the front and rear wheel speeds for both sides
    float front_left_speed = (float)((uint32_t)(CVC_data[SENSOR_LEFT_WHEELSPEED]));
    float front_right_speed = (float)((uint32_t)(CVC_data[SENSOR_RIGHT_WHEELSPEED]));
    // float rear_left_speed = (float)((uint32_t)CVC_data[INVERTER1_MOTOR_SPEED_HS_FILTERED]);
    // float rear_right_speed = (float)((uint32_t)CVC_data[INVERTER2_MOTOR_SPEED_HS_FILTERED]);
    float rear_left_speed = (float)((uint32_t)(CVC_data[INVERTER1_MOTOR_SPEED_HS]));
    float rear_right_speed = (float)((uint32_t)(CVC_data[INVERTER2_MOTOR_SPEED_HS]));

    // Update CVC_data with slip ratios of both sides and multiply by RPM_SCAlE_FACTOR

    if (rear_left_speed <= MIN_REAR_SPEED || rear_right_speed <= MIN_REAR_SPEED){
        CVC_data[CVC_SLIP_RATIO_LEFT] = 0;
        CVC_data[CVC_SLIP_RATIO_RIGHT] = 0;
    }else{
        CVC_data[CVC_SLIP_RATIO_LEFT] = (uint16_t)(((rear_left_speed - front_left_speed) / rear_left_speed)*RPM_SCALE_FACTOR);
        CVC_data[CVC_SLIP_RATIO_RIGHT] = (uint16_t)(((rear_right_speed - front_right_speed) / rear_right_speed)*RPM_SCALE_FACTOR);
    }
}