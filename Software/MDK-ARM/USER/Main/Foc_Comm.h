/*
 * @Date: 2025-02-27 19:16:35
 * @LastEditors: ZHUOZHUOO
 * @LastEditTime: 2025-11-13 22:39:33
 * @FilePath: \Software\MDK-ARM\USER\Main\Foc_Comm.h
 * @Description: Do not edit
 */
#ifndef __FOC_COMM_H
#define __FOC_COMM_H

#include "util_fdcan.h"
#include "configure.h"
#include "Foc_Control.h"
#include "Foc_Error.h"
#include "alg_pid.h"

//-----------FDCAN Mask Setting---------------//

#define FILTER_ID_1         0x500
#define FILTER_ID_2         0x7F0
#define DEVICE_ID_MASK      0x7F0
#define DEVICE_ID_NOMASK    0xFFF
#define CMD_MASK            0x00F

//-----------FDCAN Command Setting------------//

#define CMD_THETA_SET           0x001
#define CMD_CURRENT_SET         0x002
#define CMD_THETA_CURRENT_FDB   0x003
#define CMD_ERROR_FDB           0x004
#define CMD_IQ_PID_SET          0x005
#define CMD_IQ_PID_FDB          0x006
#define CMD_ID_PID_SET          0x007
#define CMD_ID_PID_FDB          0x008
#define CMD_POSITION_PID_SET    0x009
#define CMD_POSITION_PID_FDB    0x00A
#define CMD_MOTOR_ENABLE        0x00B

//-----------FOC Communication Timeout Setting------------//

#define FOC_COMM_TIMEOUT_MS    100

//--------------FOC Shutdown Enum---------------//
#define SHUTDOWN_MOTOR       0x78
#define ENABLE_MOTOR         0x91

typedef enum
{
    FOC_Comm_Timeout = 0,
    FOC_Comm_OK = 1
} FOC_Comm_Status_t;

typedef enum
{
    FOC_Comm_Shutdown = 0,
    FOC_Comm_Running = 1
} FOC_Comm_Shutdown_t;

typedef struct
{
    uint32_t last_comm_time;
    uint32_t now_time;

    FDCAN_HandleTypeDef *hfdcan;
    uint8_t RxData[64];
    FDCAN_RxHeaderTypeDef RxHeader;

    FOC_Comm_Status_t FOC_Comm_Status;
    FOC_Comm_Shutdown_t FOC_Comm_Shutdown;
} FOC_Comm_Struct;

extern FOC_Comm_Struct Motor_Comm;

void FDCAN_IntFilterAndStart(void);
void FOC_Comm_TxData_Encoder(uint32_t cmd, uint8_t *txdata);
void FOC_Comm_State_Updata(void);

#endif
