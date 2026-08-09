#ifndef __MOTOR_STATE_H
#define __MOTOR_STATE_H

#include "main.h"

#define Align_Time_ms       1000U    //转子对齐时间
#define Align_Id            0.3f    //转子对齐时给的Id值
#define Open_Loop_Iq        0.5f    
#define Open_Loop_Step      0.005f
//开环切入有感的条件:开环运行时间足够 + 电机确实已经转起来
#define OPEN_LOOP_RUN_TIME_MS              1000U
#define OPEN_LOOP_MIN_ENCODER_SPEED_RPM    60.0f

/* 编码器/SMO切换参数：角度单位rad，速度单位机械rpm。 */
#define SMO_SWITCH_UP_RPM                  500.0f         /* 高于300rpm，才允许从编码器切换到SMO */
#define SMO_SWITCH_DOWN_RPM                400.0f         /* 低于200rpm，SMO退回编码器,与300形成回差，防止抖动低速导致来回切换有感无感 */
#define SMO_SWITCH_ANGLE_ERROR_MAX_RAD     0.261799388f   /* 15deg:有感无感切换时角度偏差不能过大 */
#define SMO_RUN_ANGLE_ERROR_MAX_RAD        0.785398163f   /* 45deg:进入无感后，如果编码器与SMO角度误差超过45°，说明SMO可能跑偏，立即退回编码器 */
#define SMO_SWITCH_SPEED_ERROR_MAX_RPM     50.0f          /* 有感无感切换时，速度偏差必须小于50  */
#define SMO_SWITCH_STABLE_TIME_MS          200U           /* 切换条件必须连续稳定200ms，才真正进入SMO状态  */
#define SMO_SWITCH_STABLE_COUNT            (SMO_SWITCH_STABLE_TIME_MS * 10U)/* 防止某一瞬间角度和速度刚好满足条件，误切入SMO  */

/* 新角度源同步偏移在10kHz下约50ms时间常数，大角差时限制每拍修正速度。 */
#define ANGLE_SYNC_DECAY                   0.998f         /* 让新旧角度之间的偏移量逐拍减小 */
#define ANGLE_SYNC_MAX_STEP_RAD            0.002f         /* 每次修正角度最大值  */
#define ANGLE_SYNC_EPSILON_RAD             0.0001f        /* 角度偏移太小直接认为同步完成，将偏移清零 */


typedef enum
{
    motor_control_open_loop = 0,
    motor_control_encoder,
    motor_control_smo
}Motor_ControlMode;
extern volatile Motor_ControlMode g_motor_control_mode;//当前电机控制模式，决定了FOC使用的角度源

typedef enum
{
    motor_state_idle = 0,       // 停机
    motor_state_current_offset, // 电流零偏校准
    motor_state_align,          // 转子对齐s
    motor_state_identify,       // 参数辨识Rs/Ls 
    motor_state_open_loop,      // FOC开环转动
    motor_state_encoder_run,    // 编码器有感FOC
    motor_state_smo_run,        // SMO无感FOC
    motor_state_fault           // 故障
}Motor_State;
extern volatile Motor_State g_motor_state;
extern volatile float g_foc_elec_angle;         //保存当前真正输入FOC的电角度
extern volatile float g_encoder_smo_angle_error;//保存编码器角度和SMO角度之间的最短角度误差


void Motor_StateInit(void);
void Motor_SetState(Motor_State state);
Motor_State Motor_GetState(void);
uint8_t Motor_SetControlMode(Motor_ControlMode mode);
Motor_ControlMode Motor_GetControlMode(void);
void Motor_StateTask_1ms(void);
void Motor_FastControlISR(void);

#endif
