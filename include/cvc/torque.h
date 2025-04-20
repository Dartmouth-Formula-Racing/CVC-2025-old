/*
 * torque.h
 *
 * Created on September 20, 2024
 * Andrei Gerashchenko
 */
#ifndef CVC_TORQUE_H
#define CVC_TORQUE_H

#include <cvc/data.h>

#define TORQUE_PERIOD 3             // ms
#define NOMINAL_TORQUE 121          // Nm
#define REVERSE_TORQUE_LIMIT 30.0   // Percentage of nominal torque (0.0 - 100.0)
#define TORQUE_LIMIT 100.0          // Percentage of nominal torque (0.0 - 100.0)
#define DISABLE_ON_ZERO_THROTTLE 0  // Disable inverters when throttle is at 0%
#define INVERTER_ACCEL_AVERAGE 3    // Number of acceleration values to average
#define INVERTER_USE_HS_DATA 1      // 1 if using high-speed message, 0 if using fast messages

// RPM calculated using 20.5in OD tires & 4.7:1 gear reduction
#define REGEN_ENABLE 1           // Enable regenerative braking
#define REGEN_TORQUE_MIN 1.0     // Minimum regenerative braking torque (Nm)
#define REGEN_TORQUE_LIMIT 20.0  // Maximum regenerative braking torque (Nm)
#define REGEN_MAX_CELL_VOLTAGE 4.1  // Maximum regenerative braking cell voltage (V), 4.1V per cell
#define REGEN_MAX_CURRENT 72        // Maximum regenerative braking current (A), 6A per cell * 12 cells in parallel
#define REGEN_MIN_SPEED 240      // Minimum speed for regenerative braking (5 KPH in RPM)
#define REGEN_FULL_SPEED 1500    // Minimum speed for full regenerative braking (15 KPH in RPM)
#define REGEN_THROTTLE_ZERO 0.1  // Throttle value for switching from regen to torque command (0.0 - 1.0)

#define LEFT_MOTOR_ENABLE 1   // Left motor enable bit
#define RIGHT_MOTOR_ENABLE 1  // Right motor enable bit

#define TORQUE_VECTORING_GAIN 0.70  // Gain for torque vectoring
#define REGEN_VECTORING_GAIN -0.30  // Gain for regen torque vectoring, should be negative
#define STEERING_POT_LEFT 0.51      // Left-most position of steering potentiometer
#define STEERING_POT_RIGHT 4.03     // Right-most position of steering potentiometer

#define ACCEL_INT_FLOAT_SCALING 10000     // Scaling factor for acceleration values
#define TRACTION_CONTROL_MAX_ACCEL 0.351  // Maximum acceleration in RPM/ms
#define TRACTION_CONTROL_GAIN 0.0         // Gain for traction control

#define TORQUE_CONSTANT 0.32                // Nm/Arms
#define RPM_TO_RADS 0.1047197551            // 2 * pi / 60
#define INVERTER_CURRENT_LIMIT 310.0        // Amps
#define BUS_MIN_VOLTAGE 155.0               // Volts
#define BUS_RESISTANCE 0.2589641434         // Ohms
#define TORQUE_COMMAND_SCALE 121.0 / 100.0  // Need to command 121 Nm to inverter to get 100 Nm at motor

extern bool Inverter1_Clear_Flag;
extern bool Inverter2_Clear_Flag;

void Torque_CalculateAcceleration(void);

void Torque_CalculateTorque(void);

void Torque_SendTorque(void);

void Torque_ClearFaults(void);

void Torque_CalculateAvailableTorque(void);

#endif  // CVC_TORQUE_H