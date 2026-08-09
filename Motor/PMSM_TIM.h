#ifndef __PMSM_TIM_H
#define __PMSM_TIM_H


#include "main.h"

/* 栅极驱动SD_IN：高电平使能，低电平关断。 */
#define MOTOR_DRIVER_SD_PORT       GPIOF
#define MOTOR_DRIVER_SD_PIN        GPIO_PIN_10
#define MOTOR_DRIVER_SD_ENABLE     GPIO_PIN_SET
#define MOTOR_DRIVER_SD_DISABLE    GPIO_PIN_RESET

//上桥臂
#define  PWM_UH_PORT     GPIOA
#define  PWM_UH_PIN      GPIO_PIN_8
#define  PWM_VH_PIN      GPIO_PIN_9
#define  PWM_WH_PIN      GPIO_PIN_10
//下桥臂
#define  PWM_UL_PORT     GPIOB
#define  PWM_UL_PIN      GPIO_PIN_13
#define  PWM_VL_PIN      GPIO_PIN_14
#define  PWM_WL_PIN      GPIO_PIN_15

#define MOTOR_PWM_TIM    TIM1
#define MOTOR_PWM_PERIOD       8400U    /* PWM周期值ARR，中心对齐模式下频率=168M/(2*8400)=10kHz */
#define MOTOR_PWM_DEADTIME     100U     /* 死区时间，单位：定时器时钟周期，约595ns */

extern TIM_HandleTypeDef htim1; 


void PMSM_TIM_Init(void);
void PMSM_PWM_Duty_Set(uint16_t u_duty, uint16_t v_duty, uint16_t w_duty);
void PMSM_PWM_Start(void);
void PMSM_PWM_Stop(void);
uint8_t PMSM_Driver_ShutdownDetected(void);
void SpeedLoop_TIM6_Init(void);
void SpeedLoop_TIM6_Start(void);


#endif
