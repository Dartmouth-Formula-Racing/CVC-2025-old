/*
* misc.h
*
* Created on November 16, 2024
* Andrei Gerashchenko
*/
#ifndef CVC_MISC_H
#define CVC_MISC_H

#define BATTERY_FAN_COOLING_TEMP 25 // Celsius
#define BATTERY_FAN_HYSTERESIS 2
#define RADIATOR_FAN_COOLING_TEMP 25
#define RADIATOR_FAN_HYSTERESIS 10

/**
 * @brief Function for managing cooling system
 * @param None
 * @retval None
 */
void CVC_Cooling_Task(void);

#endif // CVC_MISC_H