#ifndef __ENCODER_H
#define __ENCODER_H

#include "main.h"

/* 引脚定义 */
#define Encoder_A_Port GPIOC
#define Encoder_A_Pin GPIO_PIN_6
#define Encoder_B_Port GPIOC
#define Encoder_B_Pin GPIO_PIN_7
#define Encoder_Z_Port GPIOE
#define Encoder_Z_Pin GPIO_PIN_6

#define Encoder_AF GPIO_AF2_TIM3
#define Encoder_TIM TIM3
#define Encoder_Period (4000U - 1U) /* 编码器线数*4，1000线编码器4倍频后4000 */
// #define PI 3.14159265358979323846f
#define MOTOR_POLE_PAIRS 4U         /* 电机极对数，根据实际电机修改 */
#define Speed_LPF_Alpha  0.2f 

#define Encoder_elec_angel_read       0U    /*    电角度已校准则只读    */
#define Encoder_elec_angel_calibrate  1U    /*    电角度0位校准使能    */
extern volatile float g_encoder_elec_angle; /*    编码器电角度,直接使用变量，避免函数获取角度，提高运行速度，工程优化，0~2π */



void Encoder_Init(void);
void Encoder_Start(void);
void Encoder_Stop(void);
int32_t Encoder_Get_Cnt(void);
void Encoder_CNT_Update_MultiTurn(void); /*    获取编码器CNT值   */
float Encoder_Angle(void);               /*    编码器获取机械角度   */
float Encoder_ElecAngle(uint8_t calibrate_enable);/*    编码器获取电角度   */
float Encoder_Speed(void);               /*    编码器计算速度：TIM6调用   */

#endif
