#ifndef __DCMOTOR_H
#define __DCMOTOR_H

#include "./SYSTEM/sys/sys.h"

/*************************************    基础驱动    *****************************************************/

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc);          /* 高级定时器 互补输出 初始化函数 */
void dcmotor_init(void);                                           /* 直流有刷电机初始化 */
void dcmotor_start(void);                                          /* 开启电机 */
void dcmotor_stop(void);                                           /* 关闭电机 */  
void dcmotor_dir(uint8_t para);                                    /* 设置电机方向 */
    
#endif


















