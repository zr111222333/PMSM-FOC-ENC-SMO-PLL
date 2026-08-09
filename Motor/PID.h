#ifndef __PID_H
#define __PID_H

#include "main.h"

typedef struct
{
    __IO float Kp;
    __IO float Ki;
    __IO float Kd;
    __IO float Kb; /* 抗饱和系数 */
    __IO float Error;
    __IO float Error_Sum;
    __IO float Error_Last;
    __IO float SetPoint;
    __IO float Output_raw;   /* 未限幅当前输出,用于抗饱和计算 */  
    __IO float Output;       /* 当前输出 */
    /* 输出限幅，防止因为目标值达不到而积分严重累计 */
    __IO float OutMax;       /* 输出上限 */
    __IO float OutMin;       /* 输出下限 */
} PID_Struct;
extern PID_Struct g_location_pid;
extern PID_Struct g_speed_pid;
extern PID_Struct g_current_pid_Id;
extern PID_Struct g_current_pid_Iq;

/* 电流环 */
#define CURRENT_LOOP_BANDWIDTH_HZ  500.0f//参数封装函数名直白明了
#define CURRENT_LOOP_OMEGA_C       3141.592654f /* 2*pi*500 */
#define CURRENT_LOOP_TS_S          0.0001f
#define I_Kp_defult         1.037f     //200Hz:0.415f  500Hz:1.037f
#define I_Ki_default        0.0927f    //200Hz:0.0371f  500Hz:0.0927f
extern volatile float g_current_Kp;
extern volatile float g_current_Ki;
#define I_Kp         (g_current_Kp)
#define I_Ki         (g_current_Ki)

#define I_Kd         0
#define I_Kb         0.20f
#define I_OutMax     8.0f
#define I_OutMin    -8.0f

/* 速度环初始参数：速度单位rpm，输出单位A */
#define S_Kp         0.0075f      //     0.0075f
#define S_Ki         0.000004f   //    0.0000 04f 
#define S_Kd         0
#define S_Kb         0.2f
#define S_OutMax     1.2f
#define S_OutMin     -1.2f

void PID_Init(PID_Struct *pid,float Kp,float Ki,float Kd,float Kb,float OutMax,float OutMin);
float PID_Ctrl(PID_Struct *pid, float Feedback_value);



#endif
