/**
 * @brief  BLDC_TIM 用于六步验证硬件  →  确认硬件OK  →  切到FOC闭环调试
 * @param  
 * @param  
 * @note   六步换相的每一步，只有一对桥臂导通，电流路径极其清晰
 *          FOC是三相同时调制的，输出的是合成矢量。如果某一相有故障，你看到的是电流环跟踪异常
 *          但定位到底是哪一相、哪个桥臂，反而需要多一步分析。
 */
#ifndef __BLDC_TIM_H
#define __BLDC_TIM_H


#include "main.h"

// //上桥臂
// #define  PWM_UH_PORT     GPIOA
// #define  PWM_UH_PIN      GPIO_PIN_8
// #define  PWM_VH_PIN      GPIO_PIN_9
// #define  PWM_WH_PIN      GPIO_PIN_10
// //下桥臂
// #define  PWM_UL_PORT     GPIOB
// #define  PWM_UL_PIN      GPIO_PIN_13
// #define  PWM_VL_PIN      GPIO_PIN_14
// #define  PWM_WL_PIN      GPIO_PIN_15

// #define MOTOR_PWM_TIM    TIM1
// #define MOTOR_PWM_PERIOD       8400U    /* PWM周期值ARR，中心对齐模式下频率=168M/(2*8400)=10kHz */
// #define MOTOR_PWM_DEADTIME     100U     /* 死区时间，单位：定时器时钟周期，约595ns */

// extern TIM_HandleTypeDef htim1; 


void BLDC_TIM_Init(void);
void BLDC_PWM_Duty_Set(uint16_t u_duty, uint16_t v_duty, uint16_t w_duty);
void BLDC_PWM_Start(void);
void BLDC_PWM_Stop(void);


#endif
