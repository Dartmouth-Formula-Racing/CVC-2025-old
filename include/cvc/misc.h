/*
* misc.h
*
* Created on November 16, 2024
* Andrei Gerashchenko
*/
#ifndef CVC_MISC_H
#define CVC_MISC_H

#define CELL_TEMP_THRESHOLD 35 // Celsius
#define CELL_TEMP_HYSTERESIS 2
#define BMS_TEMP_THRESHOLD 40
#define BMS_TEMP_HYSTERESIS 2
#define INVERTER_TEMP_THRESHOLD 30
#define INVERTER_TEMP_HYSTERESIS 10
#define MOTOR_TEMP_THRESHOLD 35
#define MOTOR_TEMP_HYSTERESIS 10

/**
 * @brief Function for managing cooling system
 * @param None
 * @retval None
 */
void CVC_Cooling_Task(void);

#endif // CVC_MISC_H