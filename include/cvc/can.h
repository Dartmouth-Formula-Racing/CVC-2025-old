/*
 * can.h
 *
 * Created on September 19, 2024
 * Andrei Gerashchenko
 */
#ifndef CVC_CAN_H
#define CVC_CAN_H

#include <stdbool.h>
#include <stdint.h>
#include <stm32f7xx_hal_can.h>

#define CAN_RX_BUFFER_LENGTH 128
#define CAN_TX_BUFFER_LENGTH 16
#define CAN_MAX_SEND_TIME 10 // ms

#define CAN_EMUS_USE_EXT 0       // 1 if using extended IDs, 0 if using standard IDs
#define CAN_EMUS_BASE_29 0x19B5  // Base ID for EMUS BMS 29-bit IDs
#define CAN_EMUS_BASE_11 0x320   // Base ID for EMUS BMS 11-bit IDs

#define CAN_VDM_USE_EXT 1        // 1 if using extended IDs, 0 if using standard IDs
#define CAN_VDM_BASE_29 0x0000A  // Base ID for VDM 29-bit IDs

#define CAN_INVERTER_USE_EXT 1          // 1 if using extended IDs, 0 if using standard IDs
#define CAN_INVERTER_BASE_ID1_11 0x0D0  // Base ID for Inverter 1
#define CAN_INVERTER_BASE_ID2_11 0x0A0  // Base ID for Inverter 2
#define CAN_INVERTER_BASE_ID1_29 0x6C0  // Base ID for Inverter 1
#define CAN_INVERTER_BASE_ID2_29 0x5C0  // Base ID for Inverter 2

#define CAN_SENSORBOARD_USE_EXT 0
#define CAN_SENSORBOARD_BASE_11 0x65D

#define CAN_DASHBOARD_USE_EXT 0
#define CAN_DASHBOARD_BASE_11 0x750

#define CAN_BROADCAST_USE_EXT 0
#define CAN_BROADCAST_BASE_11 CAN_DASHBOARD_BASE_11

#define CAN_SAFETYBROADCAST_INTERVAL 50  // ms
#define CAN_BROADCAST_INTERVAL 33        // ms

extern bool CANRxOverflow;
extern bool Inverter1_Position_Flag;
extern bool Inverter2_Position_Flag;
extern bool Inverter1_Analog_Flag;
extern bool Inverter2_Analog_Flag;
extern bool Inverter1_HS_Flag;
extern bool Inverter2_HS_Flag;


/* Struct to hold messages used in CAN message queues */
typedef struct
{
    union {
        CAN_TxHeaderTypeDef Tx_header;
        CAN_RxHeaderTypeDef Rx_header;
    };
    uint8_t data[8];
} CAN_Queue_Frame_t;

/* Circular buffer for CAN message queues */
typedef struct {
    volatile CAN_Queue_Frame_t buffer[CAN_RX_BUFFER_LENGTH];
    volatile uint16_t head;
    volatile uint16_t tail;
} RxCircularBuffer;
typedef struct {
    volatile CAN_Queue_Frame_t buffer[CAN_TX_BUFFER_LENGTH];
    volatile uint16_t head;
    volatile uint16_t tail;
} TxCircularBuffer;

void CAN_Queue_TX(CAN_Queue_Frame_t *tx_frame);
void CAN_Process_TX(void);
void CAN_Process_RX(void);
void CAN_Store_Data(uint32_t IDE, uint32_t id, uint64_t data64);
void CAN_BroadcastSafety(void);
void CAN_BroadcastData(void);
void CAN_BroadcastDebug(void);

#endif  // CVC_CAN_H