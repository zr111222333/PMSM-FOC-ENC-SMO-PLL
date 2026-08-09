#ifndef __SMO_H__
#define __SMO_H__

#include "main.h"


#ifndef PI
#define PI 3.14159265358979323846f
#endif

/* ==================== 电机参数 ==================== */
/* 电机初始参数  后续应根据实测温升和参数辨识修正。 */
#define Motor_Rs_default          0.295f    //根据电机参数设置的默认值
#define Motor_Ls_default          0.00033f
extern volatile float g_Motor_Rs;           //参数辨识检测的值
extern volatile float g_Motor_Ls;
#define Motor_Rs         (g_Motor_Rs)       //电机运行时使用的值
#define Motor_Ls         (g_Motor_Ls)
#define Motor_Flux       0.00575f
#define Motor_PolePairs  4.0f

/* 电流环频率10kHz，对应周期100us */
#define SMO_Ts           0.0001f


/* ==================== SMO参数 ==================== */
/* 滑模切换增益，单位V。增大可增强收敛，但抖振也会增大。 */
#define SMO_Ks            8.0f

/* sat函数边界层，单位A */
#define SMO_Boundary      1.0f

/* 反电动势低通滤波系数 */
#define SMO_LPF_Alpha     0.90f

/* ==================== PLL参数 ==================== */
#define PLL_Kp            50.0f
#define PLL_Ki            1000.0f

/* 机械转速低通，10kHz下截止频率约7Hz，与编码器速度带宽接近 */
#define SMO_SPEED_LPF_Alpha  0.9955f

/* 保护和阈值参数 */
#define PLL_Integral_Max  500.0f   // 根据实际调整
#define PLL_Integral_Min -500.0f
#define SMO_Min_valid_speed_elec  20.0f   // 最小有效电角速度(rad/s)，低于此值认为反电势太小，角度无效
#define SMO_Max_speed_elec        1600.0f // PLL估算电角速度最大限幅(rad/s)，防止PLL跑飞,4000*2π/60*4=1675




typedef struct
{
    float I_alpha;  /* 实测alpha轴电流 */
    float I_beta;   /* 实测beta轴电流 */
    float V_alpha;  /* 实测alpha轴电压*/
    float V_beta;   /* 实测beta轴电压 */
}SMO_Input_Def;
extern SMO_Input_Def SMO_Input;


typedef struct
{
    /* SMO主要输出 */
    float theta;          /* 反电动势直接计算的电角度，0~2pi */
    float theta_last;     /* 上一时刻的电角度 */
    float speed;          /* 反电动势幅值计算的电角速度，rad/s */
    float speed_rpm;      /* PLL估算机械转速，rpm */


    /* 反电动势 */
    float E_alpha ;         /* alpha轴反电动势 */
    float E_beta;           /* beta轴反电动势 */
    float E_alpha_lpf ;     /* 滤波后的alpha轴反电动势 */
    float E_beta_lpf;       /* 滤波后的beta轴反电动势 */
    float E_alpha_comp;     /* 相位补偿后的alpha轴反电动势 */
    float E_beta_comp;      /* 相位补偿后的beta轴反电动势 */
    uint8_t first_run;      /* 首拍标志 */ 

    /* PLL调试量 */
    float Ed;              /* 补偿反电动势在PLL d轴上的原始误差，V */
    float Ed_error;        /* Ed误差 */
    float PLL_theta;       /* PLL估算电角度，rad，0~2pi */
    float PLL_speed_delta; /* PLL PI输出的速度修正量，rad/s */
    float PLL_speed;       /* PLL估算总电角速度，rad/s */

    /* SMO内部调试量 */
    float I_alpha_hat;  /* alpha轴电流估计值 */
    float I_beta_hat;   /* beta轴电流估计值 */
    float I_alpha_err;  /* alpha轴电流估计误差 */
    float I_beta_err;   /* beta轴电流估计误差 */
    
    /* 状态与诊断标志 */
    uint8_t input_valid;    // 输入数据是否有效 (0:存在NaN/Inf异常, 1:正常)
    uint8_t angle_valid;    // 估算角度是否有效 (0:低速反电势太小失效, 1:有效)
    
}SMO_Output_Def;
extern volatile SMO_Output_Def SMO_Output;


void SMO_Init(void);
void SMO_Run(const SMO_Input_Def *input, volatile SMO_Output_Def *output);
void SMO_Observe_Run(void);


#endif
