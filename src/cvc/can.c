/*
 * can.c
 *
 * Created on September 19, 2024
 * Andrei Gerashchenko
 */

#include <cvc/can.h>
#include <cvc/data.h>
#include <cvc/parse.h>
#include <cvc/statemachine.h>
#include <main.h>
#include <stm32f7xx_hal_can.h>

CircularBuffer CANRxBuffer;
CircularBuffer CANTxBuffer;
bool CANRxOverflow = false;
bool Inverter1_Position_Flag = false;
bool Inverter2_Position_Flag = false;
bool Inverter1_Analog_Flag = false;
bool Inverter2_Analog_Flag = false;
extern CAN_HandleTypeDef hcan1;

// CAN receive interrupt, stores received frame in receive FIFO
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
    CAN_Queue_Frame_t rx_frame;
    uint16_t next_head = (CANRxBuffer.head + 1) % CAN_BUFFER_LENGTH;
    if (next_head != CANRxBuffer.tail) {  // Check for buffer full condition
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_frame.Rx_header, rx_frame.data) == HAL_OK) {
            CANRxBuffer.buffer[CANRxBuffer.head] = rx_frame;
            CANRxBuffer.head = next_head;
        }
    } else {
        CANRxOverflow = true;
    }
}

// Pulls CAN frames out of receive FIFO and stores data according to the frame's ID
// Empties receive FIFO when called
void CAN_Process_RX() {
    CAN_Queue_Frame_t rx_frame;
    __disable_irq();
    while (CANRxBuffer.head != CANRxBuffer.tail) {
        rx_frame = CANRxBuffer.buffer[CANRxBuffer.tail];
        CANRxBuffer.tail = (CANRxBuffer.tail + 1) % CAN_BUFFER_LENGTH;
        uint64_t data64 = 0;
        for (int i = 0; i < 8; i++) {
            data64 |= (uint64_t)rx_frame.data[i] << (i * 8);
        }
        if (rx_frame.Rx_header.IDE == CAN_ID_STD) {
            CAN_Store_Data(rx_frame.Rx_header.IDE, rx_frame.Rx_header.StdId, data64);
        } else {
            CAN_Store_Data(rx_frame.Rx_header.IDE, rx_frame.Rx_header.ExtId, data64);
        }
    }
    __enable_irq();
}

void CAN_Process_TX() {
    CAN_Queue_Frame_t tx_frame;
    while (CANTxBuffer.head != CANTxBuffer.tail) {
        tx_frame = CANTxBuffer.buffer[CANTxBuffer.tail];

        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0) {
            uint32_t tx_mailbox;
            HAL_CAN_AddTxMessage(&hcan1, &tx_frame.Tx_header, tx_frame.data, &tx_mailbox);
            CANTxBuffer.tail = (CANTxBuffer.tail + 1) % CAN_BUFFER_LENGTH;
        }
    }
}

void CAN_Queue_TX(CAN_Queue_Frame_t *tx_frame) {
    uint16_t next_head = (CANTxBuffer.head + 1) % CAN_BUFFER_LENGTH;
    if (next_head != CANTxBuffer.tail) {
        CANTxBuffer.buffer[CANTxBuffer.head] = *tx_frame;
        CANTxBuffer.head = next_head;
    }
}

void CAN_Store_Data(uint32_t IDE, uint32_t id, uint64_t data64) {
    if (IDE == CAN_ID_STD) {  // Standard message parsers
        if (id == ((CAN_DASHBOARD_BASE_11))) {
            // DASHBOARD Selector
            CAN_data[DASHBOARD_Selector] = data64;
        } else if (id == ((CAN_SENSORBOARD_BASE_11 + 0))) {
            // SENSORBOARD Data
            CAN_data[SENSORBOARD_Data] = data64;
        }
        if (!CAN_INVERTER_USE_EXT) {
            if (id == ((CAN_INVERTER_BASE_ID1_11 + 0))) {
                CAN_data[INVERTER1_Temp1] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 1))) {
                CAN_data[INVERTER1_Temp2] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 2))) {
                CAN_data[INVERTER1_Temp3TorqueShudder] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 3))) {
                CAN_data[INVERTER1_AnalogInputStatus] = data64;
                Inverter1_Analog_Flag = true;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 4))) {
                CAN_data[INVERTER1_DigitalInputStatus] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 5))) {
                CAN_data[INVERTER1_MotorPositionParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 6))) {
                CAN_data[INVERTER1_CurrentParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 7))) {
                CAN_data[INVERTER1_VoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 8))) {
                CAN_data[INVERTER1_FluxParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 9))) {
                CAN_data[INVERTER1_InternalVoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 10))) {
                CAN_data[INVERTER1_InternalStateParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 11))) {
                CAN_data[INVERTER1_FaultCodes] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 12))) {
                CAN_data[INVERTER1_TorqueTimerParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 13))) {
                CAN_data[INVERTER1_ModulationIndexFluxWeakening] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 14))) {
                CAN_data[INVERTER1_FirmwareInformation] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 15))) {
                CAN_data[INVERTER1_DiagnosticData] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 16))) {
                CAN_data[INVERTER1_HighSpeedParameters] = data64;
                Inverter1_Position_Flag = true;
            } else if (id == ((CAN_INVERTER_BASE_ID1_11 + 17))) {
                CAN_data[INVERTER1_TorqueCapability] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 0))) {
                CAN_data[INVERTER2_Temp1] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 1))) {
                CAN_data[INVERTER2_Temp2] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 2))) {
                CAN_data[INVERTER2_Temp3TorqueShudder] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 3))) {
                CAN_data[INVERTER2_AnalogInputStatus] = data64;
                Inverter2_Analog_Flag = true;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 4))) {
                CAN_data[INVERTER2_DigitalInputStatus] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 5))) {
                CAN_data[INVERTER2_MotorPositionParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 6))) {
                CAN_data[INVERTER2_CurrentParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 7))) {
                CAN_data[INVERTER2_VoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 8))) {
                CAN_data[INVERTER2_FluxParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 9))) {
                CAN_data[INVERTER2_InternalVoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 10))) {
                CAN_data[INVERTER2_InternalStateParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 11))) {
                CAN_data[INVERTER2_FaultCodes] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 12))) {
                CAN_data[INVERTER2_TorqueTimerParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 13))) {
                CAN_data[INVERTER2_ModulationIndexFluxWeakening] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 14))) {
                CAN_data[INVERTER2_FirmwareInformation] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 15))) {
                CAN_data[INVERTER2_DiagnosticData] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_11 + 16))) {
                CAN_data[INVERTER2_HighSpeedParameters] = data64;
                Inverter2_Position_Flag = true;
            }
        }

        if (!CAN_EMUS_USE_EXT) {
            if (id == (CAN_EMUS_BASE_11 + 0)) {
                // EMUS BMS Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x00
                CAN_data[EMUS_OverallParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 7)) {
                // EMUS BMS Diagnostic Codes - Byte 3 = 0x00 and Byte 4 = 0x07
                CAN_data[EMUS_DiagnosticCodes] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 1)) {
                // EMUS BMS Battery Voltage Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x01
                CAN_data[EMUS_BatteryVoltageOverallParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 2)) {
                // EMUS BMS Cell Module Temperature Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x02
                CAN_data[EMUS_CellModuleTemperatureOverallParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 8)) {
                // EMUS BMS Cell Temperature Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x08
                CAN_data[EMUS_CellTemperatureOverallParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 3)) {
                // EMUS BMS Cell Balancing Rate Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x03
                CAN_data[EMUS_CellBalancingRateOverallParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 5)) {
                // EMUS BMS State of Charge Parameters - Byte 3 = 0x05 and Byte 4 = 0x00
                CAN_data[EMUS_StateOfChargeParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 80)) {
                // EMUS BMS Configuration Parameters - Byte 3 = 0x04 and Byte 4 = 0x00
                CAN_data[EMUS_ConfigurationParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 81)) {
                // EMUS BMS Contactor Control - Byte 3 = 0x04 and Byte 4 = 0x01
                CAN_data[EMUS_ContactorControl] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 6)) {
                // EMUS BMS Energy Parameters - Byte 3 = 0x06 and Byte 4 = 0x00
                CAN_data[EMUS_EnergyParameters] = data64;
            } else if (id == (CAN_EMUS_BASE_11 + 85)) {
                // EMUS BMS Events - Byte 3 = 0x04 and Byte 4 = 0x05
                CAN_data[EMUS_Events] = data64;
            }
        }

    } else {  // Extended message parsers
        if (CAN_EMUS_USE_EXT) {
            if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0000)) {
                // EMUS BMS Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x00
                CAN_data[EMUS_OverallParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0007)) {
                // EMUS BMS Diagnostic Codes - Byte 3 = 0x00 and Byte 4 = 0x07
                CAN_data[EMUS_DiagnosticCodes] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0001)) {
                // EMUS BMS Battery Voltage Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x01
                CAN_data[EMUS_BatteryVoltageOverallParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0002)) {
                // EMUS BMS Cell Module Temperature Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x02
                CAN_data[EMUS_CellModuleTemperatureOverallParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0008)) {
                // EMUS BMS Cell Temperature Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x08
                CAN_data[EMUS_CellTemperatureOverallParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0003)) {
                // EMUS BMS Cell Balancing Rate Overall Parameters - Byte 3 = 0x00 and Byte 4 = 0x03
                CAN_data[EMUS_CellBalancingRateOverallParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0500)) {
                // EMUS BMS State of Charge Parameters - Byte 3 = 0x05 and Byte 4 = 0x00
                CAN_data[EMUS_StateOfChargeParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0400)) {
                // EMUS BMS Configuration Parameters - Byte 3 = 0x04 and Byte 4 = 0x00
                CAN_data[EMUS_ConfigurationParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0401)) {
                // EMUS BMS Contactor Control - Byte 3 = 0x04 and Byte 4 = 0x01
                CAN_data[EMUS_ContactorControl] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0600)) {
                // EMUS BMS Energy Parameters - Byte 3 = 0x06 and Byte 4 = 0x00
                CAN_data[EMUS_EnergyParameters] = data64;
            } else if (id == ((CAN_EMUS_BASE_29 << 16) | 0x0405)) {
                // EMUS BMS Events - Byte 3 = 0x04 and Byte 4 = 0x05
                CAN_data[EMUS_Events] = data64;
            }
        }

        if (CAN_VDM_USE_EXT) {
            if (id == ((CAN_VDM_BASE_29 << 16) | 0X0000)) {
                // VDM GPS Latitude and Longitude - 0x0000A0000
                CAN_data[VDM_GPSLatitudeLongitude] = data64;
            } else if (id == ((CAN_VDM_BASE_29 << 16) | 0X0001)) {
                // VDM GPS Data - 0x0000A0001
                CAN_data[VDM_GPSData] = data64;
            } else if (id == ((CAN_VDM_BASE_29 << 16) | 0X0002)) {
                // VDM GPS Date and Time - 0x0000A0002
                CAN_data[VDM_GPSDateTime] = data64;
            } else if (id == ((CAN_VDM_BASE_29 << 16) | 0X0003)) {
                // VDM Acceleration Data - 0x0000A0003
                CAN_data[VDM_AccelerationData] = data64;
            } else if (id == ((CAN_VDM_BASE_29 << 16) | 0X0004)) {
                // VDM Yaw Rate Data - 0x0000A0004
                CAN_data[VDM_YawRateData] = data64;
            }
        }
        if (CAN_INVERTER_USE_EXT) {
            if (id == ((CAN_INVERTER_BASE_ID1_29 + 0))) {
                CAN_data[INVERTER1_Temp1] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 1))) {
                CAN_data[INVERTER1_Temp2] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 2))) {
                CAN_data[INVERTER1_Temp3TorqueShudder] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 3))) {
                CAN_data[INVERTER1_AnalogInputStatus] = data64;
                Inverter1_Analog_Flag = true;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 4))) {
                CAN_data[INVERTER1_DigitalInputStatus] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 5))) {
                CAN_data[INVERTER1_MotorPositionParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 6))) {
                CAN_data[INVERTER1_CurrentParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 7))) {
                CAN_data[INVERTER1_VoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 8))) {
                CAN_data[INVERTER1_FluxParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 9))) {
                CAN_data[INVERTER1_InternalVoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 10))) {
                CAN_data[INVERTER1_InternalStateParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 11))) {
                CAN_data[INVERTER1_FaultCodes] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 12))) {
                CAN_data[INVERTER1_TorqueTimerParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 13))) {
                CAN_data[INVERTER1_ModulationIndexFluxWeakening] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 14))) {
                CAN_data[INVERTER1_FirmwareInformation] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 15))) {
                CAN_data[INVERTER1_DiagnosticData] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 16))) {
                CAN_data[INVERTER1_HighSpeedParameters] = data64;
                Inverter1_Position_Flag = true;
            } else if (id == ((CAN_INVERTER_BASE_ID1_29 + 17))) {
                CAN_data[INVERTER1_TorqueCapability] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 0))) {
                CAN_data[INVERTER2_Temp1] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 1))) {
                CAN_data[INVERTER2_Temp2] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 2))) {
                CAN_data[INVERTER2_Temp3TorqueShudder] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 3))) {
                CAN_data[INVERTER2_AnalogInputStatus] = data64;
                Inverter2_Analog_Flag = true;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 4))) {
                CAN_data[INVERTER2_DigitalInputStatus] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 5))) {
                CAN_data[INVERTER2_MotorPositionParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 6))) {
                CAN_data[INVERTER2_CurrentParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 7))) {
                CAN_data[INVERTER2_VoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 8))) {
                CAN_data[INVERTER2_FluxParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 9))) {
                CAN_data[INVERTER2_InternalVoltageParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 10))) {
                CAN_data[INVERTER2_InternalStateParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 11))) {
                CAN_data[INVERTER2_FaultCodes] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 12))) {
                CAN_data[INVERTER2_TorqueTimerParameters] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 13))) {
                CAN_data[INVERTER2_ModulationIndexFluxWeakening] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 14))) {
                CAN_data[INVERTER2_FirmwareInformation] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 15))) {
                CAN_data[INVERTER2_DiagnosticData] = data64;
            } else if (id == ((CAN_INVERTER_BASE_ID2_29 + 16))) {
                CAN_data[INVERTER2_HighSpeedParameters] = data64;
                Inverter2_Position_Flag = true;
            }
        }
    }
}

void CAN_BroadcastSafety() {
    static uint32_t last = 0;
    if (HAL_GetTick() - last < CAN_SAFETYBROADCAST_INTERVAL) {
        return;
    }
    last = HAL_GetTick();

    CAN_Queue_Frame_t tx_frame;
    tx_frame.Tx_header.IDE = CAN_ID_STD;
    tx_frame.Tx_header.StdId = CAN_DASHBOARD_BASE_11 + 1;
    tx_frame.Tx_header.RTR = CAN_RTR_DATA;
    tx_frame.Tx_header.DLC = 8;
    tx_frame.data[0] = CVC_data[CVC_BMS_STATE];
    tx_frame.data[1] = CVC_data[CVC_IMD_STATE];
    tx_frame.data[2] = CVC_data[CVC_DRIVE_MODE];
    tx_frame.data[3] = CVC_data[CVC_STATE];
    tx_frame.data[4] = CVC_data[CVC_BOT_STATE];
    tx_frame.data[5] = CVC_data[CVC_COCKPIT_BRB_STATE];
    tx_frame.data[6] = CVC_data[CVC_DCDC_STATE];
    CAN_Queue_TX(&tx_frame);
}

void CAN_BroadcastData() {
    static uint32_t last = 0;
    if (HAL_GetTick() - last < CAN_BROADCAST_INTERVAL) {
        return;
    }
    last = HAL_GetTick();

    // CAN_Parse_Inverter_HighSpeedParameters(0);
    // CAN_Parse_Inverter_HighSpeedParameters(1);
    CAN_Parse_Inverter_MotorPositionParameters(0);
    CAN_Parse_Inverter_MotorPositionParameters(1);

    // int16_t avg_rpm = (CVC_data[INVERTER1_MOTOR_SPEED_HS] / 2) + (CVC_data[INVERTER2_MOTOR_SPEED_HS] / 2);
    int16_t avg_rpm = 0;
    int16_t motor1_speed = (int16_t)CVC_data[INVERTER1_MOTOR_SPEED];
    int16_t motor2_speed = (int16_t)CVC_data[INVERTER2_MOTOR_SPEED];
    if (motor1_speed < 0) {
        motor1_speed = -motor1_speed;
    }
    if (motor2_speed < 0) {
        motor2_speed = -motor2_speed;
    }
    avg_rpm = motor1_speed / 2 + motor2_speed / 2;
    CAN_Queue_Frame_t tx_frame;
    tx_frame.Tx_header.IDE = CAN_ID_STD;
    tx_frame.Tx_header.StdId = CAN_DASHBOARD_BASE_11 + 2;
    tx_frame.Tx_header.RTR = CAN_RTR_DATA;
    tx_frame.Tx_header.DLC = 8;
    tx_frame.data[0] = (CVC_data[CVC_THROTTLE] >> 8) & 0xFF;
    tx_frame.data[1] = CVC_data[CVC_THROTTLE] & 0xFF;
    tx_frame.data[2] = (avg_rpm >> 8) & 0xFF;
    tx_frame.data[3] = avg_rpm & 0xFF;
    CAN_Queue_TX(&tx_frame);
}