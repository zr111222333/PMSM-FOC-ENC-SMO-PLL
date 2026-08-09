#ifndef __MOTOR_CAN_H
#define __MOTOR_CAN_H

#include "main.h"

//CAN 引脚定义
#define MOTOR_CAN_RX_PORT              GPIOI
#define MOTOR_CAN_RX_PIN               GPIO_PIN_9
#define MOTOR_CAN_RX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOI_CLK_ENABLE()
#define MOTOR_CAN_TX_PORT              GPIOB
#define MOTOR_CAN_TX_PIN               GPIO_PIN_9
#define MOTOR_CAN_TX_GPIO_CLK_ENABLE()    __HAL_RCC_GPIOB_CLK_ENABLE()
#define MOTOR_CAN_GPIO_AF              GPIO_AF9_CAN1

//CAN ID 定义 :不同ID负责不同类型的数据
#define MOTOR_CAN_ID_CONTROL           0x101U   //上位机→电机 启动、停止、模式切换
#define MOTOR_CAN_ID_SPEED             0x102U   //上位机→电机 设置目标转速
#define MOTOR_CAN_ID_STATUS            0x181U   //电机→上位机 电机状态
#define MOTOR_CAN_ID_RUNTIME           0x182U   //电机→上位机 电流、电压、实际给定
#define MOTOR_CAN_ID_PARAMETER         0x183U   //电机→上位机 Rs、Ls 辨识结果
#define MOTOR_CAN_ID_HEARTBEAT         0x184U   //电机→上位机 心跳信息
#define MOTOR_CAN_ID_RESPONSE          0x185U   //电机→上位机 命令执行结果

#define MOTOR_CAN_MAX_SPEED_RPM        2700     //最大目标转速：±2700rpm
#define MOTOR_CAN_TIMEOUT_MS           500U     //CAN控制超时：500ms

//命令枚举
typedef enum {
    MOTOR_CAN_CMD_START          = 0x01,
    MOTOR_CAN_CMD_STOP           = 0x02,
    MOTOR_CAN_CMD_SET_MODE       = 0x03,
    MOTOR_CAN_CMD_STATUS_REQUEST = 0x04,
    MOTOR_CAN_CMD_CLEAR          = 0x05,
    MOTOR_CAN_CMD_SET_SPEED      = 0x06
} Motor_CAN_Command;

//返回执行结果枚举
typedef enum {
    MOTOR_CAN_RESULT_OK     = 0,
    MOTOR_CAN_RESULT_FAILED = 1
} Motor_CAN_Result;


//返回失败原因枚举
typedef enum {
    MOTOR_CAN_REASON_NONE         = 0,
    MOTOR_CAN_REASON_FRAME_FORMAT = 1,
    MOTOR_CAN_REASON_OUT_OF_RANGE = 2,
    MOTOR_CAN_REASON_FAULT_ACTIVE = 3,
    MOTOR_CAN_REASON_STATE_DENIED = 4,
    MOTOR_CAN_REASON_MODE_INVALID = 5
} Motor_CAN_Reason;

//CAN状态错误统计结构体
typedef struct {
    uint32_t rx_error_count;    // 接收错误累计次数
    uint32_t tx_failed_count;   // 发送失败累计次数
    uint32_t bus_off_count;     // 总线关闭累计次数
    uint32_t last_error_code;   // 最后一个错误代码
} Motor_CAN_Stats;

extern CAN_HandleTypeDef hcan1;

void Motor_CAN_Init(void);
void Motor_CAN_Start(void);
void Motor_CAN_Task_1ms(void);
void Motor_CAN_SendStatus(void);
void Motor_CAN_SendHeartbeat(void);
void Motor_CAN_SendResponse(uint8_t command, uint8_t result, uint8_t reason);
void Motor_CAN_NotifyLocalControl(void);
const volatile Motor_CAN_Stats *Motor_CAN_GetStats(void);

#endif
