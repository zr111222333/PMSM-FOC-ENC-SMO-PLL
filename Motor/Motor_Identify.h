#ifndef __MOTOR_IDENTIFY_H
#define __MOTOR_IDENTIFY_H


#include "main.h"

//参数辨识状态
typedef enum {
    motor_identify_idle = 0,
    motor_identify_init,
    motor_identify_rs,
    motor_identify_ls,
    motor_identify_done,
    motor_identify_failed
} Motor_Identify_State;

//参数辨识错误原因
typedef enum {
    motor_identify_fail_none = 0,
    motor_identify_fail_not_stopped,
    motor_identify_fail_vbus,
    motor_identify_fail_current_limit,
    motor_identify_fail_rs_timeout,
    motor_identify_fail_rs_range,
    motor_identify_fail_ls_response,
    motor_identify_fail_ls_range,
    motor_identify_fail_total_timeout,
    motor_identify_fail_protection
} Motor_Identify_Fail;

//参数辨识返回结果
typedef struct {
    Motor_Identify_State State;
    Motor_Identify_Fail Fail;
    float Rs;
    float Ls;
} Motor_Identify_Result;
extern volatile Motor_Identify_Result g_motor_identify;

void Motor_Identify_Init(void);
void Motor_Identify_Start(void);
void Motor_Identify_FastTask(void);
void Motor_Identify_Abort(Motor_Identify_Fail reason);
void Motor_Identify_ReportOnce(void);
uint8_t Motor_Identify_WasAttempted(void);
uint8_t Motor_Identify_IsFinished(void);

#endif
