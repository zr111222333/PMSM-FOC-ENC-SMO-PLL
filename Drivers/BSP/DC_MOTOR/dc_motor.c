#include "./BSP/DC_MOTOR/dc_motor.h"
#include "./SYSTEM/delay/delay.h"


/*************************************    基础驱动    *****************************************************/

TIM_HandleTypeDef g_atimx_cplm_pwm_handle;      /* 定时器初始化句柄 */

/**
 * @brief       高级定时器TIMX 互补输出 初始化函数（使用PWM模式1）
 * @param       arr: 自动重装值。
 * @param       psc: 时钟预分频数
 * @retval      无
 */

void atim_timx_cplm_pwm_init(uint16_t arr, uint16_t psc)
{

    GPIO_InitTypeDef gpio_init_struct;
    TIM_OC_InitTypeDef sConfigOC ;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig ;

    /******************************* 第一步  开启定时器和IO时钟 ******************************/

    __HAL_RCC_GPIOA_CLK_ENABLE();           /* 主通道IO时钟使能 */
    __HAL_RCC_GPIOB_CLK_ENABLE();           /* 互补通道IO时钟使能 */
    __HAL_RCC_TIM1_CLK_ENABLE();            /* 定时器1时钟使能 */


    /******************************* 第二步  配置主通道、互补通道IO **************************/

    gpio_init_struct.Pin = GPIO_PIN_8;                                      /* 主通道引脚 */
    gpio_init_struct.Mode = GPIO_MODE_AF_PP;
    gpio_init_struct.Pull = GPIO_NOPULL;
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH ;
    gpio_init_struct.Alternate = GPIO_AF1_TIM1;                             /* 端口复用 */
    HAL_GPIO_Init(GPIOA, &gpio_init_struct);

    gpio_init_struct.Pin = GPIO_PIN_13;                                     /* 互补通道引脚 */
    HAL_GPIO_Init(GPIOB, &gpio_init_struct);


    /******************************* 第三步  配置定时器 **************************************/

    g_atimx_cplm_pwm_handle.Instance = TIM1;                                /* 定时器1 */
    g_atimx_cplm_pwm_handle.Init.Prescaler = psc;                           /* 定时器预分频系数 */
    g_atimx_cplm_pwm_handle.Init.CounterMode = TIM_COUNTERMODE_UP;          /* 向上计数模式 */
    g_atimx_cplm_pwm_handle.Init.Period = arr;                              /* 自动重装载值 */
    g_atimx_cplm_pwm_handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;    /* 时钟分频因子 */
    g_atimx_cplm_pwm_handle.Init.RepetitionCounter = 0;                     /* 重复计数器寄存器为0 */
    g_atimx_cplm_pwm_handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;        /* 使能影子寄存器TIMx_ARR */
    HAL_TIM_PWM_Init(&g_atimx_cplm_pwm_handle) ;


    /******************************* 第四步  配置PWM输出 *************************************/

    sConfigOC.OCMode = TIM_OCMODE_PWM1;                                     /* PWM模式1 */
    sConfigOC.Pulse = 1200;                                                 /* 预设一个比较值，给电机一个固定转速 */
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;                              /* OCy 低电平有效 */
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;                            /* OCyN 低电平有效 */
    sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;                               /* 不使用快速模式 */
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;                          /* 主通道的空闲状态 */
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;                        /* 互补通道的空闲状态 */
    HAL_TIM_PWM_ConfigChannel(&g_atimx_cplm_pwm_handle, &sConfigOC, TIM_CHANNEL_1);        /* 配置PWM通道 */


    /******************************* 第五步  配置死区 **************************************/

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;                 /* OSSR设置为1 ，输出通道无效时输出无效电平 */
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;               /* OSSI设置为0 */
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;                     /* 上电只能写一次，需要更新死区时间时只能用此值 */
    sBreakDeadTimeConfig.DeadTime = 0X0F;                                   /* 死区时间 */
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;                    /* BKE = 0, 关闭BKIN检测 */
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_LOW;             /* BKP = 1, BKIN低电平有效 */
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;     /* 使能AOE位，允许刹车后自动恢复输出 */
    HAL_TIMEx_ConfigBreakDeadTime(&g_atimx_cplm_pwm_handle, &sBreakDeadTimeConfig);         /* 设置BDTR寄存器 */

}


/**
 * @brief       电机初始化
 * @param       无
 * @retval      无
 */
void dcmotor_init(void)
{
    GPIO_InitTypeDef gpio_init_struct;
    
    __HAL_RCC_GPIOF_CLK_ENABLE();                           /* SD引脚时钟使能 */

    gpio_init_struct.Pin = GPIO_PIN_10;                     /* SD引脚 */
    gpio_init_struct.Mode = GPIO_MODE_OUTPUT_PP;            /* 推挽输出 */
    gpio_init_struct.Pull = GPIO_PULLDOWN;                  /* 下拉 */
    gpio_init_struct.Speed = GPIO_SPEED_FREQ_HIGH;          /* 高速 */
    HAL_GPIO_Init(GPIOF, &gpio_init_struct);                /* 初始化SD引脚 */
    
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);  /* 拉低SD引脚 */
}

/**
 * @brief       电机开启
 * @param       无
 * @retval      无
 */
void dcmotor_start(void)
{
    HAL_TIM_Base_Start(&g_atimx_cplm_pwm_handle);           /* 开启定时器 */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_SET);    /* 拉高SD引脚 */
}

/**
 * @brief       电机停止
 * @param       无
 * @retval      无
 */
void dcmotor_stop(void)
{
    HAL_TIM_Base_Stop(&g_atimx_cplm_pwm_handle);            /* 开启定时器 */
    HAL_GPIO_WritePin(GPIOF, GPIO_PIN_10, GPIO_PIN_RESET);  /* 拉低SD引脚 */
}

/**
 * @brief       电机旋转方向设置
 * @param       para:方向 0正转，1反转
 * @note        以电机正面，顺时针方向旋转为正转
 * @retval      无
 */
void dcmotor_dir(uint8_t para)
{
    HAL_TIM_PWM_Stop(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);            /* 关闭主通道输出 */
    HAL_TIMEx_PWMN_Stop(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);         /* 关闭互补通道输出 */
    
    if(para == 0)
    {
        HAL_TIM_PWM_Start(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);       /* 开启主通道输出 */
    }
    else if (para == 1)
    {
        HAL_TIMEx_PWMN_Start(&g_atimx_cplm_pwm_handle,TIM_CHANNEL_1);    /* 开启互补通道输出 */
    }

}




