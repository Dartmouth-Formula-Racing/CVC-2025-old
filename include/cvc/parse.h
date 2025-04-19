/*
* parse.h
*
* Created on September 19, 2024
* Andrei Gerashchenko
*/
#ifndef CVC_PARSE_H
#define CVC_PARSE_H

#include <stdint.h>
#include <stdbool.h>

// ========== Dashboard Parsing Functions ==========
/**
 * @brief Parses Dashboard 11-bit CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_Dashboard();

// ========== EMUS BMS Parsing Functions ==========

/**
 * @brief Parses EMUS BMS 29-bit Overall Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_OverallParameters();

/**
 * @brief Parses EMUS BMS 29-bit Diagnostic Codes CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_DiagnosticCodes();

/**
 * @brief Parses EMUS BMS 29-bit Battery Voltage Overall Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_BatteryVoltageOverallParameters();

/**
 * @brief Parses EMUS BMS 29-bit Cell Module Temperature Overall Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_CellModuleTemperatureOverallParameters();

/**
 * @brief Parses EMUS BMS 29-bit Cell Temperature Overall Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_CellTemperatureOverallParameters();

/**
 * @brief Parses EMUS BMS 29-bit Cell Balancing Rate Overall Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_CellBalancingRateOverallParameters();

/**
 * @brief Parses EMUS BMS 29-bit State of Charge Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_StateOfChargeParameters();

/**
 * @brief Parses EMUS BMS 29-bit Configuration Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_ConfigurationParameters();

/**
 * @brief Parses EMUS BMS 29-bit Contactor Control CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_ContactorControl();

/**
 * @brief Parses EMUS BMS 29-bit Energy Parameters CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_EnergyParameters();

/**
 * @brief Parses EMUS BMS 29-bit Events CAN message.
 * @param None
 * @retval None
 */
void CAN_Parse_EMUS_Events();

/**
 * @brief Parses VDM GPS Latitude and Longitude 29-bit CAN message (0x0000A0000).
 * @param None
 * @retval None
 */

void CAN_Parse_VDM_GPSLatitudeLongitude();

/**
 * @brief Parses VDM GPS data including speed, altitude, true course, satellites in use, and GPS validity (0x0000A0001).
 * @param None
 * @retval None
 */

void CAN_Parse_VDM_GPSData();

/**
 * @brief Parses VDM GPS date and time information (0x0000A0002).
 * @param None
 * @retval None
 */

void CAN_Parse_VDM_GPSDateTime();

/**
 * @brief Parses acceleration data for the X, Y, and Z axes (0x0000A0003).
 * @param None
 * @retval None
 */
void CAN_Parse_VDM_AccelerationData();

/**
 * @brief Parses yaw rate data for the X, Y, and Z axes (0x0000A0004).
 * @param None
 * @retval None
 */
void CAN_Parse_VDM_YawRateData();

/**
 * @brief Parse Sensor Board CAN messages.
 * @param None
 * @retval None
 */
void CAN_Parse_SensorBoard();

/**
 * @brief Parses Inverter 29-bit Temperatures #1 CAN message. (0x0A0)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_Temp1(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Temperatures #2 CAN message. (0x0A1)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_Temp2(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Temperatures #3 & Torque Shudder CAN message. (0x0A2)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_Temp3TorqueShudder(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Analog Input Status CAN message. (0x0A3)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_AnalogInputStatus(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Digital Input Status CAN message. (0x0A4)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_DigitalInputStatus(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Motor Position Parameters CAN message. (0x0A5)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_MotorPositionParameters(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Current Parameters CAN message. (0x0A6)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_CurrentParameters(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Voltage Parameters CAN message. (0x0A7)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_VoltageParameters(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Flux Parameters CAN message. (0x0A8)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_FluxParameters(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Internal Voltage Paramaters CAN message. (0x0A9)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_InternalVoltageParameters(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Internal States CAN message. (0x0AA)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_InternalStateParameters(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit Fault Codes CAN message. (0x0AB)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_FaultCodes(bool firstInverter);

/**
 * @brief Parses Inverter 29-bit High Speed CAN message. (0x0B0)
 * @param None
 * @retval None
 */

void CAN_Parse_Inverter_HighSpeedParameters(bool firstInverter);

/**
 * @brief Parses Front Sensoar Board 64-bit CAN message. (0x65D)
 * @param None
 * @retval None
 */
void CAN_Parse_SensorBoard_FilteredSpeeds();


#endif // CVC_PARSE_H