#ifndef __MOTOR_PROTECTION_H
#define __MOTOR_PROTECTION_H

#include "main.h"

//出错状态
typedef enum
{ 
    Fault_NONE            = 0,  //无故障
    Fault_Over_Current    = 1,  //过流
    Fault_Over_Voltage    = 2,  //过压
    Fault_Over_Temp       = 3,  //过温
    Fault_Under_Voltage   = 4,  //欠压
    Fault_Stall           = 5,  //堵转
    Fault_Driver_Shutdown = 6,  //驱动器关断
    Fault_Phase_Loss      = 7   //相位缺失
} Motor_Fault_e;

/* 缺相保护阈值：Motor_Protect_Check每1ms调用一次。 */
#define PHASE_LOSS_LPF_ALPHA             0.02f   //三相绝对值低通系数
#define PHASE_LOSS_SPEED_MIN_RPM        300.0f   //低于该转速禁止检测
#define PHASE_LOSS_SPEED_ERROR_MAX_RPM  100.0f   //转速尚未跟稳时禁止检测
#define PHASE_LOSS_CURRENT_MIN_A          0.15f  //其他两相有效电流下限
#define PHASE_LOSS_CURRENT_RATIO          0.30f  //缺失相与其他相的电流比例
#define PHASE_LOSS_CONFIRM_COUNT           200U  //连续约200ms才确认缺相


//阈值
typedef struct 
{
    float Over_Current_Thr;  //过流阈值
    float Over_Voltage_Thr;  //过压阈值
    float Under_Voltage_Thr; //欠压阈值
    float Over_Temp_Thr;     //过温阈值
    float Speed_Min_Rpm;     //堵转低速阈值，机械rpm
    uint32_t Stall_cnt;      //堵转持续时间
    uint32_t Stall_Thr;      //堵转阈值
    Motor_Fault_e Fault;
}Motor_Protect_t;

extern volatile Motor_Protect_t g_protect;

void Motor_Protect_Init(void);
void Motor_Protect_Check(void);
void Motor_Protect_Trip(Motor_Fault_e fault);  /*  统一故障关断函数 */
void Motor_Protect_FastCheck(void);            /*  10kHz快速保护函数，供FOC调用 */

#endif
