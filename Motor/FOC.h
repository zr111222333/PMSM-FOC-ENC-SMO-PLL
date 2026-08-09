#ifndef __FOC_H
#define __FOC_H

#include "main.h"
#include "math.h"

#ifndef PI
#define PI 3.14159265358979f
#endif

//FOC使用参数
typedef struct
{
    float Iq_ref;
    float Id_ref;
    float Tpwm;    /* PWM周期 */
    float theta;   /* 当前电角度 */
    float Udc;     /* 母线电压 */
    uint8_t Dir;   /* 电机转向 */
}FOC_Input;

//Vd、Vq电压
typedef struct { float Vd; float Vq; } Voltage_DQ;
//Vα、Vβ电压
typedef struct { float V_alpha; float V_beta; } Voltage_AlphaBeta;
//Ia、Ib、Ic3相电流
typedef struct { float Ia; float Ib; float Ic; } Current_ABC;
//Iα、Iβ电流
typedef struct { float I_alpha; float I_beta; } Current_AlphaBeta;
//Id、Iq电流
typedef struct { float Id; float Iq; } Current_DQ;

//COS_SIN计算值存储
typedef struct { float sin_theta; float cos_theta; } SinCos_Cault;


/* 外部调用变量 */
extern FOC_Input         foc_input;
extern Voltage_DQ        V_dq;
extern Voltage_AlphaBeta V_AlphaBeta;
extern Current_ABC       I_abc;
extern Current_AlphaBeta I_AlphaBeta;
extern Current_DQ        I_dq;
extern SinCos_Cault      SinCos_Transf;

void Foc_Init(void);
void Foc_Run(void);
void Foc_Run_Core(float theta);
void Foc_Run_Theta(float theta);
void Foc_Run_Voltage(float vd, float vq, float theta);//参数定时使用，直接指定Vd、Vq,不经过PI获取Vd、Vq参数更真实
void Theta_Transf_CosSin(float theta, SinCos_Cault* SinCos_Transf);
void RevPark_Transf(Voltage_DQ *V_dq_name,float theta,Voltage_AlphaBeta *V_AlphaBeta_name);
void Clark_Transf(Current_ABC *I_abc_name,Current_AlphaBeta *I_AlphaBeta_name);
void Park_Transf(Current_AlphaBeta *I_AlphaBeta_name,float theta,Current_DQ *I_dq_name);
void SVPWM_Transf(Voltage_AlphaBeta *V_AlphaBeta_name,float Udc,float Tpwm);


#endif
