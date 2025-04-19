/*
Author: Sasha Ries
Date: 2/10/25
File: traction_control.h
Decription: Header file for traction_control.c
*/

#define MIN_REAR_SPEED 1
#define TRACTION_CONTROL_SMP_RATE 10

// Traction control state variables - declare as static at the module level
typedef struct {
    float optimal_slip;      // Target slip ratio for best traction
    float error_previous;    // Previous error for derivative calculation
    float error_integral;    // Accumulated error for integral term
    float kp;                // Proportional gain
    float ki;                // Integral gain
    float kd;                // Derivative gain
    float output_limit_min;  // Minimum output limit (0.0)
    float output_limit_max;  // Maximum output limit (1.0)
    float integral_limit;    // Anti-windup limit for integral term
    float sample_time_s;     // Sample time in seconds (0.01 for 10ms)
} TractionControl_t;

/**
 * Calculates slip ratios for left and right wheels using front and rear wheel speeds.
 * Stores scaled slip ratios (×RPM_SCALE_FACTOR) in CVC_data.
 * Sets values to zero when rear speeds are below minimum threshold.
 *
 * @reads CVC_data wheel speed values
 * @writes CVC_SLIP_RATIO_LEFT, CVC_SLIP_RATIO_RIGHT
 */
void Traction_Calculate_SlipRatio(void);

/**
 * Closed-loop digital traction control algorithm that applies PID control
 * to maintain optimal slip ratio.
 *
 * @param tc Pointer to the traction control state structure
 * @param current_slip The current wheel slip ratio
 * @return Torque scaling factor between 0 and 1
 */
float Traction_Control_Update(TractionControl_t *tc, float current_slip);

/**
 * Reset the traction control state when needed
 * (e.g., when entering a new drive mode or after being disabled)
 * @param tc Pointer to the traction control state structure
 * @return void
 */
void Traction_Control_Reset(TractionControl_t *tc);