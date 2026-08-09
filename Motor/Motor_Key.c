#include "Motor_Key.h"

//按键初始化
void Motor_Key_Init(void)
{
    __HAL_RCC_GPIOE_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init_struct;

    gpio_init_struct.Mode=GPIO_MODE_INPUT;      /* 输入 */
    gpio_init_struct.Pin=KEY0_GPIO_PIN;         /* KEY0引脚 */
    gpio_init_struct.Pull=GPIO_PULLUP;          /* 上拉 */
    gpio_init_struct.Speed=GPIO_SPEED_FREQ_HIGH;/* 高速 */
    HAL_GPIO_Init(KEY_GPIO_PORT,&gpio_init_struct);/* KEY0引脚模式设置,上拉输入 */

    gpio_init_struct.Pin=KEY1_GPIO_PIN;         /* KEY1引脚 */
    HAL_GPIO_Init(KEY_GPIO_PORT,&gpio_init_struct);/* KEY1引脚模式设置,上拉输入 */
    
    gpio_init_struct.Pin=KEY2_GPIO_PIN;         /* KEY2引脚 */
    HAL_GPIO_Init(KEY_GPIO_PORT,&gpio_init_struct);/* KEY2引脚模式设置,上拉输入 */ 
}

/**
 * @brief       按键扫描函数
 * @note        该函数有响应优先级(同时按下多个按键): WK_UP > KEY2 > KEY1 > KEY0!!
 * @param       mode:0 / 1, 具体含义如下:
 *   @arg       0,  不支持连续按(当按键按下不放时, 只有第一次调用会返回键值,
 *                  必须松开以后, 再次按下才会返回其他键值)
 *   @arg       1,  支持连续按(当按键按下不放时, 每次调用该函数都会返回键值)
 * @retval      键值, 定义如下:
 *              KEY0_PRES, 1, KEY0按下
 *              KEY1_PRES, 2, KEY1按下
 *              KEY2_PRES, 3, KEY2按下
 *              WKUP_PRES, 4, WKUP按下
 */
uint8_t Motor_Key_Detect(uint8_t mode)
{
    static uint8_t key_up=1;      //按键松开标志
    uint8_t keyval=0;           //按键检测返回值

    if(mode)   key_up=1;        //支持连按
    if(key_up && (KEY0==0 || KEY1==0 || KEY2==0))    //按键松开标志为1，有任意一个按键按下
    {
        delay_ms(10);   //消抖
        key_up=0;
        
        if(KEY0==0)     keyval=1;

        if(KEY1==0)     keyval=2;

        if(KEY2==0)     keyval=3;
    }
    else if(KEY0==1 && KEY1==1 && KEY2==1)  //没按键按下，标记按键松开
    {
        key_up=1; 
    }

    return keyval;
}
