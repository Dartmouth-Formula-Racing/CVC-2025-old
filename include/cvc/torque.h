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
#define REVERSE_TORQUE_LIMIT 20.0   // Percentage of nominal torque (0.0 - 100.0)
#define TORQUE_LIMIT 100.0          // Percentage of nominal torque (0.0 - 100.0)
#define DISABLE_ON_ZERO_THROTTLE 0  // Disable inverters when throttle is at 0%
#define INVERTER_ACCEL_AVERAGE 3    // Number of acceleration values to average
#define INVERTER_USE_HS_DATA 1      // 1 if using high-speed message, 0 if using fast messages

#define LEFT_MOTOR_ENABLE 1   // Left motor enable bit
#define RIGHT_MOTOR_ENABLE 1  // Right motor enable bit

#define TORQUE_VECTORING_GAIN 0.70  // Gain for torque vectoring
#define STEERING_POT_LEFT 0.51      // Left-most position of steering potentiometer
#define STEERING_POT_RIGHT 4.03     // Right-most position of steering potentiometer

#define ACCEL_INT_FLOAT_SCALING 10000     // Scaling factor for acceleration values
#define TRACTION_CONTROL_MAX_ACCEL 0.351  // Maximum acceleration in RPM/ms
#define TRACTION_CONTROL_GAIN 0.0         // Gain for traction control

#define TORQUE_CONSTANT 0.32                // Nm/Arms
#define RPM_TO_RADS 0.1047197551            // 2 * pi / 60
#define INVERTER_CURRENT_LIMIT 310.0        // Amps
#define BUS_MIN_VOLTAGE 155.0               // Volts
#define CELL_INTERNAL_RESISTANCE 0.013      // Ohms
#define BATTERY_CELLS_PARALLEL 12           // Number of cells in parallel
#define BATTERY_CELLS_SERIES 60             // Number of cells in series
#define CONTACTOR_RESISTANCE 0.0002         // Ohms
#define MISC_RESISTANCE 0.002               // Ohms
#define BUS_RESISTANCE 0.2589641434         // Ohms
#define TORQUE_COMMAND_SCALE 121.0 / 100.0  // Need to command 121 Nm to inverter to get 100 Nm at motor
#define PACK_RESISTANCE (BATTERY_CELLS_SERIES * (CELL_INTERNAL_RESISTANCE / BATTERY_CELLS_PARALLEL) + CONTACTOR_RESISTANCE * 2 + MISC_RESISTANCE)

extern bool Inverter1_Clear_Flag;
extern bool Inverter2_Clear_Flag;

void Torque_CalculateAcceleration(void);

void Torque_CalculateTorque(void);

void Torque_SendTorque(void);

void Torque_ClearFaults(void);

void Torque_CalculateAvailableTorque(void);

#endif  // CVC_TORQUE_H