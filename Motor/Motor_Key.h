#ifndef __MOTOR_KEY_H__
#define __MOTOR_KEY_H__


#include "main.h"

#define KEY_GPIO_PORT    GPIOE
#define KEY0_GPIO_PIN     GPIO_PIN_2
#define KEY1_GPIO_PIN     GPIO_PIN_3
#define KEY2_GPIO_PIN     GPIO_PIN_4


// #define KEY0    HAL_GPIO_ReadPin(KEY_GPIO_PORT,KEY0_GPIO_PIN)  /* 读取KEY0引脚 */
// #define KEY1    HAL_GPIO_ReadPin(KEY_GPIO_PORT,KEY1_GPIO_PIN) /* 读取KEY1引脚 */
// #define KEY2    HAL_GPIO_ReadPin(KEY_GPIO_PORT,KEY2_GPIO_PIN) /* 读取KEY2引脚 */

void Motor_Key_Init(void);                   //按键初始化
uint8_t Motor_Key_Detect(uint8_t mode);     //按键扫描函数


#endif
