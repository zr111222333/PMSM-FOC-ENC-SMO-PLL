

#ifndef __KEY_H
#define __KEY_H

#include "./SYSTEM/sys/sys.h"

/******************************************************************************************/
/* 引脚 定义 */

#define KEY0_GPIO_PORT                  GPIOE
#define KEY0_GPIO_PIN                   GPIO_PIN_2
#define KEY0_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define KEY1_GPIO_PORT                  GPIOE
#define KEY1_GPIO_PIN                   GPIO_PIN_3
#define KEY1_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

#define KEY2_GPIO_PORT                  GPIOE
#define KEY2_GPIO_PIN                   GPIO_PIN_4
#define KEY2_GPIO_CLK_ENABLE()          do{ __HAL_RCC_GPIOE_CLK_ENABLE(); }while(0)   /* PE口时钟使能 */

/******************************************************************************************/

#define KEY0        HAL_GPIO_ReadPin(KEY0_GPIO_PORT, KEY0_GPIO_PIN)     /* 读取KEY0引脚 */
#define KEY1        HAL_GPIO_ReadPin(KEY1_GPIO_PORT, KEY1_GPIO_PIN)     /* 读取KEY1引脚 */
#define KEY2        HAL_GPIO_ReadPin(KEY2_GPIO_PORT, KEY2_GPIO_PIN)     /* 读取KEY2引脚 */

#define KEY0_PRES    1              /* KEY0按下 */
#define KEY1_PRES    2              /* KEY1按下 */
#define KEY2_PRES    3              /* KEY2按下 */

void key_init(void);                /* 按键初始化函数 */
uint8_t key_scan(uint8_t mode);     /* 按键扫描函数 */

#endif


















