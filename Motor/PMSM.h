#ifndef __PMSM_H
#define __PMSM_H

#include "main.h"

/* 旋转方向 */
#define CW     1    /* 顺时针 */
#define CCW    0    /* 逆时针 */

/* 速度环每5ms允许变化2rpm，即400rpm/s；可按机械负载调整。 */
#define SPEED_REF_RAMP_STEP_RPM  2.0f



extern volatile float g_motor_speed;
extern volatile float g_speed_ref_rpm;
extern volatile float g_speed_ref_now_rpm;//堵转保护需要知道速度斜坡是否还没有执行完成
extern volatile float g_PMSM_Speed_NowUse_rpm;/*   用于PMSM速度环当前究竟使用编码器速度还是SMO速度   */
// #define g_speed_feedback_rpm g_PMSM_Speed_NowUse_rpm/*   用于PMSM速度环当前究竟使用编码器速度还是SMO速度   */

void PMSM_Init(void);
void Motor_Six_Step_SetPhase(uint8_t step,uint16_t duty);   //单步六步换相PWM输出
void Motor_Six_Step_Run(uint8_t dir, uint16_t duty);        //开环强拖：连续六步换相运行
void Motor_Six_Step_Stop(void);                             //停止开环强拖的六步换相，所有桥臂关闭
void Motor_SetSpeedRef(float speed_ref_rpm);                //带符号速度判断方向：正数正转，负数反转
void Motor_SetDirection(uint8_t dir);                      //由方向判断速度：运行中切换方向，速度环负责制动后反转



#endif
