#ifndef __ADC_H
#define __ADC_H

#include "main.h"

#define VBUS_DIVIDER_GAIN    25.0f  // 电压分压系数，母线电压=ADC采样值*3.3/4096*25
#define ADC_REF_VOLTAGE       3.3f
#define ADC_FULL_SCALE     4096.0f


typedef struct
{
    float Iu_raw;    /* U相ADC原始值减去零偏后的差值 */
    float Iv_raw;    /* V相ADC原始值减去零偏后的差值 */
    float Iw_raw;    /* W相ADC原始值减去零偏后的差值 */
    float Vbus_raw;  /* 母线电压原始值 */
    float Temp_raw;  /* 温度原始值 */
} MotorADC_RawTypeDef;
extern MotorADC_RawTypeDef ADC_Raw;

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern MotorADC_RawTypeDef ADC_Raw;


void Motor_ADC_RCC_GPIO_DMA_Init(void);
void Motor_ADC1_Init(void);
void Motor_ADC2_Init(void);
void Motor_ADC3_Init(void);
void Motor_ADC_Init(void);
void Motor_ADC_Start(void);
void Motor_ADC_Get_Current_Offset(void);               // 电流零偏校准
void Motor_ADC_Get_RawValue(MotorADC_RawTypeDef *raw); // 获取三相电流+母线+温度原始值
float Motor_GetTemperature(void);

#endif
