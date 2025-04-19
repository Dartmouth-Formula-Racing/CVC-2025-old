#include <cvc/data.h>
#include <cvc/parse.h>

// ========== Dashboard Parsing Functions ==========
/**
 * @brief Parses Dashboard 11-bit CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_Dashboard() {
    if (CAN_data_parsed[DASHBOARD_Selector]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[DASHBOARD_Selector] >> (i * 8)) & 0xFF;
    }
    CVC_data[DASH_REQUESTED_STATE] = data[0];
    CAN_data_parsed[DASHBOARD_Selector] = true;
}

// TODO: Implement 11-bit CAN message parsing functions
// Only 29-bit CAN message parsing functions are currently implemented
// ========== EMUS BMS Parsing Functions ==========

/**
 * @brief Parses EMUS BMS 29-bit Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_OverallParameters() {
    if (CAN_data_parsed[EMUS_OverallParameters]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_OverallParameters] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Input signals
    // Bit 0: Ignition key
    // Bit 1: Charger mains
    // Bit 2: Fast charge
    // Bit 3: Leakage sensor
    // Bits 4-7: Don't care
    CVC_data[BMS_IGNITION] = (data[0] >> 0) & 0x01;
    CVC_data[BMS_CHARGER_MAINS] = (data[0] >> 1) & 0x01;
    CVC_data[BMS_FAST_CHARGE] = (data[0] >> 2) & 0x01;
    CVC_data[BMS_LEAKAGE_DETECTED] = (data[0] >> 3) & 0x01;
    // Byte 1: Output signals
    // Bit 0: Charger enable
    // Bit 1: Heater enable
    // Bit 2: Battery contactor
    // Bit 3: Battery fan
    // Bit 4: Power reduction
    // Bit 5: Charging interlock
    // Bit 6: DCDC control
    // Bit 7: Contactor precharge
    CVC_data[BMS_CHARGER_ENABLED] = (data[1] >> 0) & 0x01;
    CVC_data[BMS_HEATER_ENABLED] = (data[1] >> 1) & 0x01;
    CVC_data[BMS_BATTERY_CONTACTOR] = (data[1] >> 2) & 0x01;
    CVC_data[BMS_BATTERY_FAN] = (data[1] >> 3) & 0x01;
    CVC_data[BMS_POWER_REDUCTION] = (data[1] >> 4) & 0x01;
    CVC_data[BMS_CHARGING_INTERLOCK] = (data[1] >> 5) & 0x01;
    CVC_data[BMS_DCDC_ENABLED] = (data[1] >> 6) & 0x01;
    CVC_data[BMS_CONTACTOR_PRECHARGE] = (data[1] >> 7) & 0x01;
    // Byte 2: Number of live cells (MSB)
    // Byte 3: Charging state
    // 0 = Disconnected
    // 1 = Pre-heating
    // 2 = Pre-charging
    // 3 = Main charging
    // 4 = Balancing
    // 5 = Charging finished
    // 6 = Charging error
    CVC_data[BMS_CHARGING_STATE] = data[3];
    // Byte 4: Charging stage duration (MSB)
    // Byte 5: Charging stage duration (LSB)
    CVC_data[BMS_CHARGING_DURATION] = (data[4] << 8) | data[5];
    // Charging stage duration is in minutes
    // Byte 6: Last charging error
    // 0 = No error
    // 1 = No cell communication at start of charging or communication lost during pre-charging (using CAN charger)
    // 2 = No cell communication using non-CAN charger
    // 3 = Maximum chargin stage duration exceeded
    // 4 = Cell communication lost during charging (CAN charger)
    // 5 = Cannot set cell module balancing threshold
    // 6 = Cell or cell module temperature too high
    // 7 = Cell commmunication lost during pre-heating (CAN charger)
    CVC_data[BMS_LAST_CHARGING_ERROR] = data[6];
    // Byte 7: Number of live cells (LSB)
    CVC_data[BMS_LIVE_CELL_COUNT] = (data[2] << 8) | data[7];
    CAN_data_parsed[EMUS_OverallParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Diagnostic Codes CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_DiagnosticCodes() {
    if (CAN_data_parsed[EMUS_DiagnosticCodes]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_DiagnosticCodes] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Protection flags (LSB)
    // Bit 0: Cell undervoltage
    // Bit 1: Cell overvoltage
    // Bit 2: Discharge overcurrent
    // Bit 3: Charge overcurrent
    // Bit 4: Cell module overheat
    // Bit 5: Leakage
    // Bit 6: No cell communication
    // Bit 7: Don't care
    CVC_data[BMS_UNDERVOLTAGE] = (data[0] >> 0) & 0x01;
    CVC_data[BMS_OVERVOLTAGE] = (data[0] >> 1) & 0x01;
    CVC_data[BMS_DISCHARGE_OVERCURRENT] = (data[0] >> 2) & 0x01;
    CVC_data[BMS_CHARGE_OVERCURRENT] = (data[0] >> 3) & 0x01;
    CVC_data[BMS_CELL_MODULE_OVERHEAT] = (data[0] >> 4) & 0x01;
    CVC_data[BMS_LEAKAGE] = (data[0] >> 5) & 0x01;
    CVC_data[BMS_NO_CELL_COMMUNICATION] = (data[0] >> 6) & 0x01;
    // Byte 1: Warning (power reduction) flags
    // Bit 0: Low cell voltage
    // Bit 1: High discharge current
    // Bit 2: High cell module temperature
    CVC_data[BMS_WARN_LOW_CELL_VOLTAGE] = (data[1] >> 0) & 0x01;
    CVC_data[BMS_WARN_HIGH_DISCHARGE_CURRENT] = (data[1] >> 1) & 0x01;
    CVC_data[BMS_WARN_HIGH_CELL_MODULE_TEMP] = (data[1] >> 2) & 0x01;
    // Byte 3: Protection flags (MSB)
    // Bit 3: Cell overheat
    // Bit 4: No current sensor
    // Bit 5: Pack undervoltage
    CVC_data[BMS_CELL_OVERHEAT] = (data[3] >> 3) & 0x01;
    CVC_data[BMS_NO_CURRENT_SENSOR] = (data[3] >> 4) & 0x01;
    CVC_data[BMS_PACK_UNDERVOLTAGE] = (data[3] >> 5) & 0x01;
    // Byte 4: Battery status flags
    // Bit 0: Cell voltages valid
    // Bit 1: Cell module temperatures valid
    // Bit 2: Cell balancing rates valid
    // Bit 3: Cell balancing thresholds valid
    // Bit 4: Charging finished
    // Bit 5: Cell temperatures valid
    CVC_data[BMS_CELL_VOLTAGE_VALID] = (data[4] >> 0) & 0x01;
    CVC_data[BMS_CELL_MODULE_TEMP_VALID] = (data[4] >> 1) & 0x01;
    CVC_data[BMS_BALANCE_RATE_VALID] = (data[4] >> 2) & 0x01;
    CVC_data[BMS_LIVE_CELL_COUNT_VALID] = (data[4] >> 3) & 0x01;
    CVC_data[BMS_CHARGING_FINISHED] = (data[4] >> 4) & 0x01;
    CVC_data[BMS_CELL_TEMP_VALID] = (data[4] >> 5) & 0x01;

    CAN_data_parsed[EMUS_DiagnosticCodes] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Battery Voltage Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_BatteryVoltageOverallParameters() {
    if (CAN_data_parsed[EMUS_BatteryVoltageOverallParameters]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_BatteryVoltageOverallParameters] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Min cell voltage
    CVC_data[BMS_MIN_CELL_VOLTAGE] = data[0] + 200;
    // Byte 1: Max cell voltage
    CVC_data[BMS_MAX_CELL_VOLTAGE] = data[1] + 200;
    // Byte 2: Average cell voltage
    CVC_data[BMS_AVG_CELL_VOLTAGE] = data[2] + 200;
    // Byte 4: Total voltage (LSB)
    CVC_data[BMS_TOTAL_VOLTAGE] = data[4] & 0xFF;
    // Byte 3: Total voltage (2nd byte)
    CVC_data[BMS_TOTAL_VOLTAGE] = data[3] << 8 | CVC_data[BMS_TOTAL_VOLTAGE];
    // Byte 6: Total voltage (3rd byte)
    CVC_data[BMS_TOTAL_VOLTAGE] = data[6] << 16 | CVC_data[BMS_TOTAL_VOLTAGE];
    // Byte 5: Total voltage (MSB)
    CVC_data[BMS_TOTAL_VOLTAGE] = data[5] << 24 | CVC_data[BMS_TOTAL_VOLTAGE];
    // Byte 7: Don't care

    CAN_data_parsed[EMUS_BatteryVoltageOverallParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Cell Module Temperature Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellModuleTemperatureOverallParameters() {
    if (CAN_data_parsed[EMUS_CellModuleTemperatureOverallParameters]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_CellModuleTemperatureOverallParameters] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Min cell module temperature
    CVC_data[BMS_MIN_CELL_MODULE_TEMP] = data[0];
    // Byte 1: Max cell module temperature
    CVC_data[BMS_MAX_CELL_MODULE_TEMP] = data[1];
    // Byte 2: Average cell module temperature
    CVC_data[BMS_AVG_CELL_MODULE_TEMP] = data[2];
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care

    CAN_data_parsed[EMUS_CellModuleTemperatureOverallParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Cell Temperature Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellTemperatureOverallParameters() {
    if (CAN_data_parsed[EMUS_CellTemperatureOverallParameters]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_CellTemperatureOverallParameters] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Min cell temperature
    CVC_data[BMS_MIN_CELL_TEMP] = data[0];
    // Byte 1: Max cell temperature
    CVC_data[BMS_MAX_CELL_TEMP] = data[1];
    // Byte 2: Average cell temperature
    CVC_data[BMS_AVG_CELL_TEMP] = data[2];
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care

    CAN_data_parsed[EMUS_CellTemperatureOverallParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Cell Balancing Rate Overall Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_CellBalancingRateOverallParameters() {
    if (CAN_data_parsed[EMUS_CellBalancingRateOverallParameters]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_CellBalancingRateOverallParameters] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Min cell balancing rate
    CVC_data[BMS_MIN_BALANCE_RATE] = data[0];
    // Byte 1: Max cell balancing rate
    CVC_data[BMS_MAX_BALANCE_RATE] = data[1];
    // Byte 2: Average cell balancing rate
    CVC_data[BMS_AVG_BALANCE_RATE] = data[2];
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care

    CAN_data_parsed[EMUS_CellBalancingRateOverallParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit State of Charge Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_StateOfChargeParameters() {
    if (CAN_data_parsed[EMUS_StateOfChargeParameters]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_StateOfChargeParameters] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Current (MSB)
    // Byte 1: Current (LSB)
    CVC_data[BMS_CURRENT] = (data[0] << 8) | data[1];
    // Byte 2: Estimated charge (MSB)
    // Byte 3: Estimated charge (LSB)
    CVC_data[BMS_ESTIMATED_CHARGE] = (data[2] << 8) | data[3];
    // Byte 4: Don't care
    // Byte 5: Estimated user state of charge (MSB)
    // Byte 6: Estimated user state of charge (LSB)
    CVC_data[BMS_ESTIMATED_SOC] = (data[5] << 8) | data[6];
    // Byte 7: Estimated state of health
    CVC_data[BMS_ESTIMATED_SOH] = data[7];

    CAN_data_parsed[EMUS_StateOfChargeParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Configuration Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_ConfigurationParameters() {
    if (CAN_data_parsed[EMUS_ConfigurationParameters]) {
        return;
    }
    // TODO: Implement
    CAN_data_parsed[EMUS_ConfigurationParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Contactor Control CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_ContactorControl() {
    if (CAN_data_parsed[EMUS_ContactorControl]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_ContactorControl] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Contactor state
    CVC_data[BMS_CONTACTOR_STATE] = data[0];
    // Byte 1: Don't care
    // Byte 2: Don't care
    // Byte 3: Don't care
    // Byte 4: Don't care
    // Byte 5: Don't care
    // Byte 6: Don't care
    // Byte 7: Don't care

    CAN_data_parsed[EMUS_ContactorControl] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Energy Parameters CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_EnergyParameters() {
    if (CAN_data_parsed[EMUS_EnergyParameters]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[EMUS_EnergyParameters] >> (i * 8)) & 0xFF;
    }

    // Byte 0: Estimated Consumption (MSB)
    // Byte 1: Estimated Consumption (LSB)
    CVC_data[BMS_ESTIMATED_CONSUMPTION] = (data[0] << 8) | data[1];
    // Byte 2: Estimated Energy (MSB)
    // Byte 3: Estimated Energy (LSB)
    CVC_data[BMS_ESTIMATED_ENERGY] = (data[2] << 8) | data[3];
    // Byte 4: Estimated Distance Remaining (MSB)
    // Byte 5: Estimated Distance Remaining (LSB)
    CVC_data[BMS_ESTIMATED_DISTANCE_REMAINING] = (data[4] << 8) | data[5];
    // Byte 6: Distance Traveled (MSB)
    // Byte 7: Distance Traveled (LSB)
    CVC_data[BMS_DISTANCE_TRAVELED] = (data[6] << 8) | data[7];

    CAN_data_parsed[EMUS_EnergyParameters] = true;
}

/**
 * @brief Parses EMUS BMS 29-bit Events CAN message.
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_EMUS_Events() {
    if (CAN_data_parsed[EMUS_Events]) {
        return;
    }
    // TODO: Implement
    CAN_data_parsed[EMUS_Events] = true;
}

/**
 * @brief Parses VDM GPS Latitude and Longitude 29-bit CAN message (0x0000A0000).
 * @param : A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSLatitudeLongitude() {
    if (CAN_data_parsed[VDM_GPSLatitudeLongitude]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[VDM_GPSLatitudeLongitude] >> (i * 8)) & 0xFF;
    }

    // Byte 0-3: GPS Latitude
    CVC_data[VDM_GPS_LATITUDE] = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    // Byte 4-7: GPS Longitude
    CVC_data[VDM_GPS_LONGITUDE] = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];

    CAN_data_parsed[VDM_GPSLatitudeLongitude] = true;
}

/**
 * @brief Parses VDM GPS data including speed, altitude, true course, satellites in use, and GPS validity (0x0000A0001).
 * @param : A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSData() {
    if (CAN_data_parsed[VDM_GPSData]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[VDM_GPSData] >> (i * 8)) & 0xFF;
    }

    // Byte 0,1: GPS Speed
    CVC_data[VDM_GPS_SPEED] = (data[0] << 8) | data[1];
    // Byte 2, 3: GPS Altitude
    CVC_data[VDM_GPS_ALTITUDE] = (data[2] << 8) | data[3];
    // Byte 4, 5: GPS True Course
    CVC_data[VDM_GPS_TRUE_COURSE] = (data[4] << 8) | data[5];
    // Byte 6: Satellites in use
    CVC_data[VDM_GPS_SATELLITES_IN_USE] = data[6];
    // Byte 7: GPS Validity
    CVC_data[VDM_GPS_DATA_VALID] = data[7];

    CAN_data_parsed[VDM_GPSData] = true;
}

/**
 * @brief Parses VDM GPS date and time information (0x0000A0002).
 * @param : A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */

void CAN_Parse_VDM_GPSDateTime() {
    if (CAN_data_parsed[VDM_GPSDateTime]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[VDM_GPSDateTime] >> (i * 8)) & 0xFF;
    }

    // Byte 0: GPS Validity
    CVC_data[VDM_GPS_DATE_TIME_VALID] = data[0];
    // Byte 1: UTC Year
    CVC_data[VDM_UTC_YEAR] = data[1];
    // Byte 2: UTC Month
    CVC_data[VDM_UTC_MONTH] = data[2];
    // Byte 3: UTC Day
    CVC_data[VDM_UTC_DAY] = data[3];
    // Byte 5: UTC Hour
    CVC_data[VDM_UTC_HOUR] = data[5];
    // Byte 6: UTC Minute
    CVC_data[VDM_UTC_MINUTE] = data[6];
    // Byte 7: UTC Second
    CVC_data[VDM_UTC_SECOND] = data[7];

    CAN_data_parsed[VDM_GPSDateTime] = true;
}

/**
 * @brief Parses acceleration data for the X, Y, and Z axes (0x0000A0003).
 * @param : A CAN frame containing bytes of acceleration data.
 * @retval None
 */
void CAN_Parse_VDM_AccelerationData() {
    if (CAN_data_parsed[VDM_AccelerationData]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[VDM_AccelerationData] >> (i * 8)) & 0xFF;
    }

    // Byte 0, 1: X-Axis Acceleration
    CVC_data[VDM_ACCELERATION_X] = (data[0] << 8) | data[1];
    // Byte 2, 3: Y-Axis Acceleration
    CVC_data[VDM_ACCELERATION_Y] = (data[2] << 8) | data[3];
    // Byte 4, 5: Z-Axis Acceleration
    CVC_data[VDM_ACCELERATION_Z] = (data[4] << 8) | data[5];

    CAN_data_parsed[VDM_AccelerationData] = true;
}

/**
 * @brief Parses yaw rate data for the X, Y, and Z axes (0x0000A0004).
 * @param : A CAN frame containing bytes of yaw rate data.
 * @retval None
 */
void CAN_Parse_VDM_YawRateData() {
    if (CAN_data_parsed[VDM_YawRateData]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[VDM_YawRateData] >> (i * 8)) & 0xFF;
    }

    // Byte 0, 1: X-Axis Yaw Rate
    CVC_data[VDM_YAW_RATE_X] = (data[0] << 8) | data[1];
    // Byte 2, 3: Y-Axis Yaw Rate
    CVC_data[VDM_YAW_RATE_Y] = (data[2] << 8) | data[3];
    // Byte 4, 5: Z-Axis Yaw Rate
    CVC_data[VDM_YAW_RATE_Z] = (data[4] << 8) | data[5];

    CAN_data_parsed[VDM_YawRateData] = true;
}

/**
 * @brief Parse Sensor Board CAN messages.
 * @param frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */
void CAN_Parse_SensorBoard_RawSpeeds() {
    if (CAN_data_parsed[SENSOR_WheelSpeed_Raw]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[SENSOR_WheelSpeed_Raw] >> (i * 8)) & 0xFF;
    }

    // Byte 0 - 3: Right Wheel  RPM
    CVC_data[SENSOR_LEFT_WHEELSPEED_RAW] = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    // Byte 4-7: Left Wheel RPM
    CVC_data[SENSOR_RIGHT_WHEELSPEED_RAW] = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];

    CAN_data_parsed[SENSOR_WheelSpeed_Raw] = true;
}

/**
 * @brief Parse Sensor Board CAN messages.
 * @param frame: A CAN frame containing 8 bytes of data to be parsed.
 * @retval None
 */
void CAN_Parse_SensorBoard_FilteredSpeeds() {
    if (CAN_data_parsed[SENSOR_WheelSpeed_Filtered]) {
        return;
    }
    uint8_t data[8] = {0};
    for (uint8_t i = 0; i < 8; i++) {
        data[i] = (CAN_data[SENSOR_WheelSpeed_Filtered] >> (i * 8)) & 0xFF;
    }

    // Byte 0 - 3: Right Wheel filtered RPM
    CVC_data[SENSOR_LEFT_WHEELSPEED] = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3];
    // Byte 4-7: Left Wheel filtered RPM
    CVC_data[SENSOR_RIGHT_WHEELSPEED] = (data[4] << 24) | (data[5] << 16) | (data[6] << 8) | data[7];

    CAN_data_parsed[SENSOR_WheelSpeed_Filtered] = true;
}

/**
 * @brief Parses Inverter 29-bit Temperatures #1 CAN message. (0)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */
void CAN_Parse_Inverter_Temp1(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_Temp1]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_Temp1] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Power Module Phase A Temperature
        CVC_data[INVERTER1_POWER_MODULE_A_TEMP] = (data[1] << 8) | data[0];
        // Byte 2, 3: Power Module Phase B Temperature
        CVC_data[INVERTER1_POWER_MODULE_B_TEMP] = (data[3] << 8) | data[2];
        // Byte 4, 5: Power Module Phase  C Temperature
        CVC_data[INVERTER1_POWER_MODULE_C_TEMP] = (data[5] << 8) | data[4];
        // Byte 6, 7: Gate Driver Board Temperature
        CVC_data[INVERTER1_GATE_DRIVER_BOARD_TEMP] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_Temp1] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_Temp1]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_Temp1] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Power Module Phase A Temperature
        CVC_data[INVERTER2_POWER_MODULE_A_TEMP] = (data[1] << 8) | data[0];
        // Byte 2, 3: Power Module Phase B Temperature
        CVC_data[INVERTER2_POWER_MODULE_B_TEMP] = (data[3] << 8) | data[2];
        // Byte 4, 5: Power Module Phase  C Temperature
        CVC_data[INVERTER2_POWER_MODULE_C_TEMP] = (data[5] << 8) | data[4];
        // Byte 6, 7: Gate Driver Board Temperature
        CVC_data[INVERTER2_GATE_DRIVER_BOARD_TEMP] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_Temp1] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Temperatures #2 CAN message. (1)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_Temp2(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_Temp2]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_Temp2] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Temperature of Control Board
        CVC_data[INVERTER1_CONTROL_BOARD_TEMP] = (data[1] << 8) | data[0];
        // Byte 2, 3: Temperature read from RTD input #1
        CVC_data[INVERTER1_RTD_INPUT_1_TEMP] = (data[3] << 8) | data[2];
        // Byte 4, 5: Temperature read from RTD Input #2
        CVC_data[INVERTER1_RTD_INPUT_2_TEMP] = (data[5] << 8) | data[4];
        // Byte 6, 7: Stall Burst Model Temperature
        CVC_data[INVERTER1_STALL_BURST_MODEL_TEMP] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_Temp2] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_Temp2]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_Temp2] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Temperature of Control Board
        CVC_data[INVERTER1_CONTROL_BOARD_TEMP] = (data[1] << 8) | data[0];
        // Byte 2, 3: Temperature read from RTD input #1
        CVC_data[INVERTER1_RTD_INPUT_1_TEMP] = (data[3] << 8) | data[2];
        // Byte 4, 5: Temperature read from RTD Input #2
        CVC_data[INVERTER1_RTD_INPUT_2_TEMP] = (data[5] << 8) | data[4];
        // Byte 6, 7: Stall Burst Model Temperature
        CVC_data[INVERTER1_STALL_BURST_MODEL_TEMP] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_Temp2] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Temperatures #3 & Torque Shudder CAN message. (2)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_Temp3TorqueShudder(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_Temp3TorqueShudder]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_Temp3TorqueShudder] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Coolant Temperature
        CVC_data[INVERTER1_COOLANT_TEMP] = (data[1] << 8) | data[0];
        // Byte 2, 3: Estimated hot spot temperature internal to inverter.
        CVC_data[INVERTER1_HOT_SPOT_TEMP] = (data[3] << 8) | data[2];
        // Byte 4, 5: Filtered temperature value from the motor temperature sensor
        CVC_data[INVERTER1_MOTOR_TEMP] = (data[5] << 8) | data[4];
        // Byte 6, 7: A value of torque used in shudder compensation.
        CVC_data[INVERTER1_TORQUE_SHUDDER] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_Temp3TorqueShudder] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_Temp3TorqueShudder]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_Temp3TorqueShudder] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Coolant Temperature
        CVC_data[INVERTER2_COOLANT_TEMP] = (data[1] << 8) | data[0];
        // Byte 2, 3: Estimated hot spot temperature internal to inverter.
        CVC_data[INVERTER2_HOT_SPOT_TEMP] = (data[3] << 8) | data[2];
        // Byte 4, 5: Filtered temperature value from the motor temperature sensor
        CVC_data[INVERTER2_MOTOR_TEMP] = (data[5] << 8) | data[4];
        // Byte 6, 7: A value of torque used in shudder compensation.
        CVC_data[INVERTER2_TORQUE_SHUDDER] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_Temp3TorqueShudder] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Analog Input Status CAN message. (0x0A3)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

// 01234567 89012345 67890123 45678901 23456789 01234567 89012345 67890123
void CAN_Parse_Inverter_AnalogInputStatus(bool inverter) {
    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_AnalogInputStatus]) {
            return;
        }
        uint64_t data64 = CAN_data[INVERTER1_AnalogInputStatus];
        // Bits 0-9: Analog Input #1
        CVC_data[INVERTER1_ANALOG_INPUT_1] = (data64 >> 0) & 0x3FF;
        // Bits 10-19: Analog Input #2
        CVC_data[INVERTER1_ANALOG_INPUT_2] = (data64 >> 10) & 0x3FF;
        // Bits 20-29: Analog Input #3
        CVC_data[INVERTER1_ANALOG_INPUT_3] = (data64 >> 20) & 0x3FF;
        // Bits 32-41: Analog Input #4
        CVC_data[INVERTER1_ANALOG_INPUT_4] = (data64 >> 32) & 0x3FF;
        // Bits 42-51: Analog Input #5
        CVC_data[INVERTER1_ANALOG_INPUT_5] = (data64 >> 42) & 0x3FF;
        // Bits 52-61: Analog Input #6
        CVC_data[INVERTER1_ANALOG_INPUT_6] = (data64 >> 52) & 0x3FF;
        CAN_data_parsed[INVERTER1_AnalogInputStatus] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_AnalogInputStatus]) {
            return;
        }
        uint64_t data64 = CAN_data[INVERTER2_AnalogInputStatus];
        // Bits 0-9: Analog Input #1
        CVC_data[INVERTER2_ANALOG_INPUT_1] = (data64 >> 0) & 0x3FF;
        // Bits 10-19: Analog Input #2
        CVC_data[INVERTER2_ANALOG_INPUT_2] = (data64 >> 10) & 0x3FF;
        // Bits 20-29: Analog Input #3
        CVC_data[INVERTER2_ANALOG_INPUT_3] = (data64 >> 20) & 0x3FF;
        // Bits 32-41: Analog Input #4
        CVC_data[INVERTER2_ANALOG_INPUT_4] = (data64 >> 32) & 0x3FF;
        // Bits 42-51: Analog Input #5
        CVC_data[INVERTER2_ANALOG_INPUT_5] = (data64 >> 42) & 0x3FF;
        // Bits 52-61: Analog Input #6
        CVC_data[INVERTER2_ANALOG_INPUT_6] = (data64 >> 52) & 0x3FF;
        CAN_data_parsed[INVERTER2_AnalogInputStatus] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Digital Input Status CAN message. (4)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_DigitalInputStatus(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_DigitalInputStatus]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_DigitalInputStatus] >> (i * 8)) & 0xFF;
        }
        // Byte 0: Status of Digital Input #1, Forward switch
        CVC_data[INVERTER1_FORWARD_SWITCH] = data[1];
        // Byte 1: Status of Digital Input #2, Reverse switch
        CVC_data[INVERTER1_REVERSE_SWITCH] = data[0];
        // Byte 2: Status of Digital Input #3, Reverse switch
        CVC_data[INVERTER1_BRAKE_SWITCH] = data[3];
        // Byte 3: Status of Digital Input #4, Regen Disable switch
        CVC_data[INVERTER1_REGEN_DISABLE_SWITCH] = data[2];
        // Byte 4: Status of Digital Input #5, Ignition switch
        CVC_data[INVERTER1_IGNITION_SWITCH] = data[5];
        // Byte 5: Status of Digital Input #6, Start switch
        CVC_data[INVERTER1_START_SWITCH] = data[4];
        // Byte 6: Status of Digital Input #7, Valet Mode
        CVC_data[INVERTER1_VALET_MODE] = data[7];
        CAN_data_parsed[INVERTER1_DigitalInputStatus] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_DigitalInputStatus]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_DigitalInputStatus] >> (i * 8)) & 0xFF;
        }
        // Byte 0: Status of Digital Input #1, Forward switch
        CVC_data[INVERTER2_FORWARD_SWITCH] = data[1];
        // Byte 1: Status of Digital Input #2, Reverse switch
        CVC_data[INVERTER2_REVERSE_SWITCH] = data[0];
        // Byte 2: Status of Digital Input #3, Reverse switch
        CVC_data[INVERTER2_BRAKE_SWITCH] = data[3];
        // Byte 3: Status of Digital Input #4, Regen Disable switch
        CVC_data[INVERTER2_REGEN_DISABLE_SWITCH] = data[2];
        // Byte 4: Status of Digital Input #5, Ignition switch
        CVC_data[INVERTER2_IGNITION_SWITCH] = data[5];
        // Byte 5: Status of Digital Input #6, Start switch
        CVC_data[INVERTER2_START_SWITCH] = data[4];
        // Byte 6: Status of Digital Input #7, Valet Mode
        CVC_data[INVERTER2_VALET_MODE] = data[7];
        CAN_data_parsed[INVERTER2_DigitalInputStatus] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Motor Position Parameters CAN message. (5)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_MotorPositionParameters(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_MotorPositionParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_MotorPositionParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Motor Angle
        CVC_data[INVERTER1_MOTOR_ANGLE] = (data[1] << 8) | data[0];
        // Byte 2, 3: Motor Speed
        CVC_data[INVERTER1_MOTOR_SPEED] = (data[3] << 8) | data[2];
        // Byte 4, 5: Electrical Output Frequency
        CVC_data[INVERTER1_ELECTRICAL_OUTPUT_FREQUENCY] = (data[5] << 8) | data[4];
        // Byte 6, 7: Delta Resolver Filtered
        CVC_data[INVERTER1_DELTA_RESOLVER_FILTERED] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_MotorPositionParameters] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_MotorPositionParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_MotorPositionParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Motor Angle
        CVC_data[INVERTER2_MOTOR_ANGLE] = (data[1] << 8) | data[0];
        // Byte 2, 3: Motor Speed
        CVC_data[INVERTER2_MOTOR_SPEED] = (data[3] << 8) | data[2];
        // Byte 4, 5: Electrical Output Frequency
        CVC_data[INVERTER2_ELECTRICAL_OUTPUT_FREQUENCY] = (data[5] << 8) | data[4];
        // Byte 6, 7: Delta Resolver Filtered
        CVC_data[INVERTER2_DELTA_RESOLVER_FILTERED] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_MotorPositionParameters] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Current Parameters CAN message. (6)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_CurrentParameters(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_CurrentParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_CurrentParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Phase A Current
        CVC_data[INVERTER1_PHASE_A_CURRENT] = (data[1] << 8) | data[0];
        // Byte 2, 3: Phase B Current
        CVC_data[INVERTER1_PHASE_B_CURRENT] = (data[3] << 8) | data[2];
        // Byte 4, 5: Phase C Current
        CVC_data[INVERTER1_PHASE_C_CURRENT] = (data[5] << 8) | data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER1_DC_BUS_CURRENT] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_CurrentParameters] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_CurrentParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_CurrentParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Phase A Current
        CVC_data[INVERTER2_PHASE_A_CURRENT] = (data[1] << 8) | data[0];
        // Byte 2, 3: Phase B Current
        CVC_data[INVERTER2_PHASE_B_CURRENT] = (data[3] << 8) | data[2];
        // Byte 4, 5: Phase C Current
        CVC_data[INVERTER2_PHASE_C_CURRENT] = (data[5] << 8) | data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER2_DC_BUS_CURRENT] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_CurrentParameters] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Voltage Parameters CAN message. (7)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_VoltageParameters(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_VoltageParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_VoltageParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: The actual measured value of the DC bus voltage
        CVC_data[INVERTER1_DC_BUS_VOLTAGE] = (data[1] << 8) | data[0];
        // Byte 2, 3: The calculated value of the output voltage, in peak line-neutral volts
        CVC_data[INVERTER1_OUTPUT_VOLTAGE] = (data[3] << 8) | data[2];
        // Byte 4, 5: Measured value of the voltage between Phase A and Phase B (VAB) when the inverter is disabled.
        // Vd voltage when the inverter is enabled.
        CVC_data[INVERTER1_VAB_VD_VOLTAGE] = (data[5] << 8) | data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER1_VAB_VQ_VOLTAGE] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_VoltageParameters] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_VoltageParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_VoltageParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: The actual measured value of the DC bus voltage
        CVC_data[INVERTER2_DC_BUS_VOLTAGE] = (data[1] << 8) | data[0];
        // Byte 2, 3: The calculated value of the output voltage, in peak line-neutral volts
        CVC_data[INVERTER2_OUTPUT_VOLTAGE] = (data[3] << 8) | data[2];
        // Byte 4, 5: Measured value of the voltage between Phase A and Phase B (VAB) when the inverter is disabled.
        // Vd voltage when the inverter is enabled.
        CVC_data[INVERTER2_VAB_VD_VOLTAGE] = (data[5] << 8) | data[4];
        // Byte 6, 7: DC BUS Current
        CVC_data[INVERTER2_VAB_VQ_VOLTAGE] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_VoltageParameters] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Flux Parameters CAN message. (8)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_FluxParameters(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_FluxParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_FluxParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: The commanded flux
        CVC_data[INVERTER1_FLUX_COMMAND] = (data[1] << 8) | data[0];
        // Byte 2, 3: The estimated flux
        CVC_data[INVERTER1_FLUX_FEEDBACK] = (data[3] << 8) | data[2];
        // Byte 4, 5: D-axis current feedback
        CVC_data[INVERTER1_ID_CURRENT] = (data[5] << 8) | data[4];
        // Byte 6, 7: Q-axis current feedback
        CVC_data[INVERTER1_IQ_CURRENT] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_FluxParameters] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_FluxParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_FluxParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: The commanded flux
        CVC_data[INVERTER2_FLUX_COMMAND] = (data[1] << 8) | data[0];
        // Byte 2, 3: The estimated flux
        CVC_data[INVERTER2_FLUX_FEEDBACK] = (data[3] << 8) | data[2];
        // Byte 4, 5: D-axis current feedback
        CVC_data[INVERTER2_ID_CURRENT] = (data[5] << 8) | data[4];
        // Byte 6, 7: Q-axis current feedback
        CVC_data[INVERTER2_IQ_CURRENT] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_FluxParameters] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Internal Voltage Paramaters CAN message. (9)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_InternalVoltageParameters(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_InternalVoltageParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_InternalVoltageParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: 1.5 Reference Voltage
        CVC_data[INVERTER1_1_5_REFERENCE_VOLTAGE] = (data[1] << 8) | data[0];
        // Byte 2, 3: 2.5 Reference Voltage
        CVC_data[INVERTER1_2_5_REFERENCE_VOLTAGE] = (data[3] << 8) | data[2];
        // Byte 4, 5: 5.0 Reference Voltage
        CVC_data[INVERTER1_5_0_REFERENCE_VOLTAGE] = (data[5] << 8) | data[4];
        // Byte 6, 7: 12.0 Reference Voltage
        CVC_data[INVERTER1_12_0_REFERENCE_VOLTAGE] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_InternalVoltageParameters] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_InternalVoltageParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_InternalVoltageParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: 1.5 Reference Voltage
        CVC_data[INVERTER2_1_5_REFERENCE_VOLTAGE] = (data[1] << 8) | data[0];
        // Byte 2, 3: 2.5 Reference Voltage
        CVC_data[INVERTER2_2_5_REFERENCE_VOLTAGE] = (data[3] << 8) | data[2];
        // Byte 4, 5: 5.0 Reference Voltage
        CVC_data[INVERTER2_5_0_REFERENCE_VOLTAGE] = (data[5] << 8) | data[4];
        // Byte 6, 7: 12.0 Reference Voltage
        CVC_data[INVERTER2_12_0_REFERENCE_VOLTAGE] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_InternalVoltageParameters] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Internal States CAN message. (10)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_InternalStateParameters(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_InternalStateParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_InternalStateParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0: VSM State
        CVC_data[INVERTER1_VSM_STATE] = data[1];
        // Byte 1: PWM Frequency
        CVC_data[INVERTER1_PWM_FREQUENCY] = data[0];
        // Byte 2: Inverter State
        CVC_data[INVERTER1_STATE] = data[3];
        // Byte 3: Relay state
        // Bit 0: Relay 1 status
        // Bit 1: Relay 2 status
        // Bit 2: Relay 3 status
        // Bit 3: Relay 4 status
        // Bit 4: Relay 5 status
        // Bit 5: Relay 6 status
        CVC_data[INVERTER1_RELAY_1_STATUS] = (data[2] >> 0) & 0x01;
        CVC_data[INVERTER1_RELAY_2_STATUS] = (data[2] >> 1) & 0x01;
        CVC_data[INVERTER1_RELAY_3_STATUS] = (data[2] >> 2) & 0x01;
        CVC_data[INVERTER1_RELAY_4_STATUS] = (data[2] >> 3) & 0x01;
        CVC_data[INVERTER1_RELAY_5_STATUS] = (data[2] >> 4) & 0x01;
        CVC_data[INVERTER1_RELAY_6_STATUS] = (data[2] >> 5) & 0x01;
        // Byte 4
        // Bit 0: Inverter Run Mode (0 = Torque Mode, 1 = Speed Mode)
        // Bit 1: Self Sensing Assist Enable
        // Bit 5-7: Inverter Active Discharge State (000 (0) = Discharge Disabled, 001 (1) = Discharge Enabled, waiting, 010 (2) = Performing Speed Check, 011
        // (3) = Discharge Actively occurring, 100 (4) = Discharge Completed)
        CVC_data[INVERTER1_RUN_MODE] = (data[5] >> 0) & 0x01;
        CVC_data[INVERTER1_SELF_SENSING_ASSIST] = (data[5] >> 1) & 0x01;
        CVC_data[INVERTER1_ACTIVE_DISCHARGE_STATE] = (data[5] >> 5) & 0x07;
        // Byte 5
        // Bit 0: Inverter Command Mode
        // Bit 4-7: Rolling Counter Value
        CVC_data[INVERTER1_COMMAND_MODE] = (data[4] >> 0) & 0x01;
        CVC_data[INVERTER1_ROLLING_COUNTER_VALUE] = (data[4] >> 4) & 0x0F;
        // Byte 6
        // Bit 0: Inverter Enable State (0 = Inverter is disabled, 1 = Inverter is enabled)
        // Bit 1: Burst Model Mode (0 = Stall, 1 = High Speed)
        // Bit 6: Start Mode Active (0 = start signal has not been activated, 1 = start signal has been activated)
        // Bit 7: Inverter Enable Lockout (0 = Inverter can be enabled, 1 = Inverter cannot be enabled)
        CVC_data[INVERTER1_ENABLE_STATE] = (data[7] >> 0) & 0x01;
        CVC_data[INVERTER1_BURST_MODEL_MODE] = (data[7] >> 0) & 0x01;
        CVC_data[INVERTER1_START_MODE_ACTIVE] = (data[7] >> 6) & 0x01;
        CVC_data[INVERTER1_ENABLE_LOCKOUT] = (data[7] >> 7) & 0x01;
        // Byte 7
        // Bit 0: Direction Command (1 = Forward ;0 = Reverse, if inverter is enabled Stopped, if inverter is disabled)
        // Bit 1: BMS Active (0 = BMS Message is not being received, 1 = BMS Message is being received)
        // Bit 2: BMS Limiting Torque ( 0 = Torque is not being limited by the BMS. 1 = Torque is being limited by the BMS.)
        // Bit 3: Limit Max Speed (0 = no torque limiting is occurring. 1 = torque limiting is occurring due to the motor speed exceeding the maximum motor
        // speed) Bit 4: Limit Hot Spot (0 = Inverter hot spot temperature is below the limit. 1 = Inverter is limiting current due to regulate the maximum hot
        // spot temperature.) Bit 5: Low Speed Limiting (0 = low speed current limiting is not occurring. 1 = low speed current limiting is applied.) Bit 6:
        // Coolant Temperature Limiting (0 = Max motor current not limited due to coolant temperature.  1 = Max motor current limited due to coolan
        // temperature.) Bit 7: Limit Stall Burst Model (0 = Not limiting. 1 = Limiting due to stall burst model.)
        CVC_data[INVERTER1_DIRECTION_COMMAND] = (data[6] >> 0) & 0x01;
        CVC_data[INVERTER1_BMS_ACTIVE] = (data[6] >> 1) & 0x01;
        CVC_data[INVERTER1_BMS_LIMITING_TORQUE] = (data[6] >> 2) & 0x01;
        CVC_data[INVERTER1_LIMIT_MAX_SPEED] = (data[6] >> 3) & 0x01;
        CVC_data[INVERTER1_LIMIT_HOT_SPOT] = (data[6] >> 4) & 0x01;
        CVC_data[INVERTER1_LOW_SPEED_LIMITING] = (data[6] >> 5) & 0x01;
        CVC_data[INVERTER1_COOLANT_TEMP_LIMITING] = (data[6] >> 6) & 0x01;
        CVC_data[INVERTER1_LIMIT_STALL_BURST_MODEL] = (data[6] >> 7) & 0x01;
        CAN_data_parsed[INVERTER1_InternalStateParameters] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_InternalStateParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_InternalStateParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0: VSM State
        CVC_data[INVERTER2_VSM_STATE] = data[1];
        // Byte 1: PWM Frequency
        CVC_data[INVERTER2_PWM_FREQUENCY] = data[0];
        // Byte 2: Inverter State
        CVC_data[INVERTER2_STATE] = data[3];
        // Byte 3: Relay state
        // Bit 0: Relay 1 status
        // Bit 1: Relay 2 status
        // Bit 2: Relay 3 status
        // Bit 3: Relay 4 status
        // Bit 4: Relay 5 status
        // Bit 5: Relay 6 status
        CVC_data[INVERTER2_RELAY_1_STATUS] = (data[2] >> 0) & 0x01;
        CVC_data[INVERTER2_RELAY_2_STATUS] = (data[2] >> 1) & 0x01;
        CVC_data[INVERTER2_RELAY_3_STATUS] = (data[2] >> 2) & 0x01;
        CVC_data[INVERTER2_RELAY_4_STATUS] = (data[2] >> 3) & 0x01;
        CVC_data[INVERTER2_RELAY_5_STATUS] = (data[2] >> 4) & 0x01;
        CVC_data[INVERTER2_RELAY_6_STATUS] = (data[2] >> 5) & 0x01;
        // Byte 4
        // Bit 0: Inverter Run Mode (0 = Torque Mode, 1 = Speed Mode)
        // Bit 1: Self Sensing Assist Enable
        // Bit 5-7: Inverter Active Discharge State (000 (0) = Discharge Disabled, 001 (1) = Discharge Enabled, waiting, 010 (2) = Performing Speed Check, 011
        // (3) = Discharge Actively occurring, 100 (4) = Discharge Completed)
        CVC_data[INVERTER2_RUN_MODE] = (data[5] >> 0) & 0x01;
        CVC_data[INVERTER2_SELF_SENSING_ASSIST] = (data[5] >> 1) & 0x01;
        CVC_data[INVERTER2_ACTIVE_DISCHARGE_STATE] = (data[5] >> 5) & 0x07;
        // Byte 5
        // Bit 0: Inverter Command Mode
        // Bit 4-7: Rolling Counter Value
        CVC_data[INVERTER2_COMMAND_MODE] = (data[4] >> 0) & 0x01;
        CVC_data[INVERTER2_ROLLING_COUNTER_VALUE] = (data[4] >> 4) & 0x0F;
        // Byte 6
        // Bit 0: Inverter Enable State (0 = Inverter is disabled, 1 = Inverter is enabled)
        // Bit 1: Burst Model Mode (0 = Stall, 1 = High Speed)
        // Bit 6: Start Mode Active (0 = start signal has not been activated, 1 = start signal has been activated)
        // Bit 7: Inverter Enable Lockout (0 = Inverter can be enabled, 1 = Inverter cannot be enabled)
        CVC_data[INVERTER2_ENABLE_STATE] = (data[7] >> 0) & 0x01;
        CVC_data[INVERTER2_BURST_MODEL_MODE] = (data[7] >> 0) & 0x01;
        CVC_data[INVERTER2_START_MODE_ACTIVE] = (data[7] >> 6) & 0x01;
        CVC_data[INVERTER2_ENABLE_LOCKOUT] = (data[7] >> 7) & 0x01;
        // Byte 7
        // Bit 0: Direction Command (1 = Forward ;0 = Reverse, if inverter is enabled Stopped, if inverter is disabled)
        // Bit 1: BMS Active (0 = BMS Message is not being received, 1 = BMS Message is being received)
        // Bit 2: BMS Limiting Torque ( 0 = Torque is not being limited by the BMS. 1 = Torque is being limited by the BMS.)
        // Bit 3: Limit Max Speed (0 = no torque limiting is occurring. 1 = torque limiting is occurring due to the motor speed exceeding the maximum motor
        // speed) Bit 4: Limit Hot Spot (0 = Inverter hot spot temperature is below the limit. 1 = Inverter is limiting current due to regulate the maximum hot
        // spot temperature.) Bit 5: Low Speed Limiting (0 = low speed current limiting is not occurring. 1 = low speed current limiting is applied.) Bit 6:
        // Coolant Temperature Limiting (0 = Max motor current not limited due to coolant temperature.  1 = Max motor current limited due to coolan
        // temperature.) Bit 7: Limit Stall Burst Model (0 = Not limiting. 1 = Limiting due to stall burst model.)
        CVC_data[INVERTER2_DIRECTION_COMMAND] = (data[6] >> 0) & 0x01;
        CVC_data[INVERTER2_BMS_ACTIVE] = (data[6] >> 1) & 0x01;
        CVC_data[INVERTER2_BMS_LIMITING_TORQUE] = (data[6] >> 2) & 0x01;
        CVC_data[INVERTER2_LIMIT_MAX_SPEED] = (data[6] >> 3) & 0x01;
        CVC_data[INVERTER2_LIMIT_HOT_SPOT] = (data[6] >> 4) & 0x01;
        CVC_data[INVERTER2_LOW_SPEED_LIMITING] = (data[6] >> 5) & 0x01;
        CVC_data[INVERTER2_COOLANT_TEMP_LIMITING] = (data[6] >> 6) & 0x01;
        CVC_data[INVERTER2_LIMIT_STALL_BURST_MODEL] = (data[6] >> 7) & 0x01;
        CAN_data_parsed[INVERTER2_InternalStateParameters] = true;
    }
}

/**
 * @brief Parses Inverter 29-bit Fault Codes CAN message. (11)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_FaultCodes(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_FaultCodes]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_FaultCodes] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: POST Fault Lo
        CVC_data[INVERTER1_POST_FAULT_LO] = (data[1] << 8) | data[0];
        // Byte 2, 3: POST Fault Hi
        CVC_data[INVERTER1_POST_FAULT_HI] = (data[3] << 8) | data[2];
        // Byte 4, 5: Run Fault Lo
        CVC_data[INVERTER1_RUN_FAULT_LO] = (data[5] << 8) | data[4];
        // Byte 6, 7: Run Fault Hi
        CVC_data[INVERTER1_RUN_FAULT_HI] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_FaultCodes] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_FaultCodes]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_FaultCodes] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: POST Fault Lo
        CVC_data[INVERTER2_POST_FAULT_LO] = (data[1] << 8) | data[0];
        // Byte 2, 3: POST Fault Hi
        CVC_data[INVERTER2_POST_FAULT_HI] = (data[3] << 8) | data[2];
        // Byte 4, 5: Run Fault Lo
        CVC_data[INVERTER2_RUN_FAULT_LO] = (data[5] << 8) | data[4];
        // Byte 6, 7: Run Fault Hi
        CVC_data[INVERTER2_RUN_FAULT_HI] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_FaultCodes] = true;
    }
}
/**
 * @brief Parses Inverter 29-bit High Speed CAN message. (0x0B0)
 * @param uint8_t data[8]: Array of 8 bytes of data to be parsed
 * @retval None
 */

void CAN_Parse_Inverter_HighSpeedParameters(bool inverter) {
    uint8_t data[8] = {0};

    if (inverter == 0) {
        if (CAN_data_parsed[INVERTER1_HighSpeedParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER1_HighSpeedParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Torque Command
        CVC_data[INVERTER1_TORQUE_COMMAND_HS] = (data[1] << 8) | data[0];
        CVC_data[INVERTER1_TORQUE_FEEDBACK_HS] = (data[3] << 8) | data[2];
        CVC_data[INVERTER1_MOTOR_SPEED_HS] = (data[5] << 8) | data[4];
        CVC_data[INVERTER1_DC_BUS_VOLTAGE_HS] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER1_HighSpeedParameters] = true;
    } else {
        if (CAN_data_parsed[INVERTER2_HighSpeedParameters]) {
            return;
        }
        for (uint8_t i = 0; i < 8; i++) {
            data[i] = (CAN_data[INVERTER2_HighSpeedParameters] >> (i * 8)) & 0xFF;
        }
        // Byte 0, 1: Torque Command
        CVC_data[INVERTER2_TORQUE_COMMAND_HS] = (data[1] << 8) | data[0];
        CVC_data[INVERTER2_TORQUE_FEEDBACK_HS] = (data[3] << 8) | data[2];
        CVC_data[INVERTER2_MOTOR_SPEED_HS] = (data[5] << 8) | data[4];
        CVC_data[INVERTER2_DC_BUS_VOLTAGE_HS] = (data[7] << 8) | data[6];
        CAN_data_parsed[INVERTER2_HighSpeedParameters] = true;
    }
}
