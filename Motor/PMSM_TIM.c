#include "PMSM_TIM.h"

TIM_HandleTypeDef htim1; // 定时器句柄
static volatile uint8_t s_driver_enable_requested = 0U;//记录软件当前是否要求驱动开启


void PMSM_PWM_Duty_Set(uint16_t u_duty, uint16_t v_duty, uint16_t w_duty);

//TIM1 : 三相 PWM (CH1/2/3 + 互补) + CH4 触发 ADC 注入
void PMSM_TIM_Init(void)
{
    /* 开启时钟 */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_GPIOF_CLK_ENABLE();   //输出使能SD引脚

    /* 上桥臂 PA8/PA9/PA10 -> AF1 */
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    //初始化输出使能SD引脚
    GPIO_InitStruct.Pin=MOTOR_DRIVER_SD_PIN;
    GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull=GPIO_NOPULL;
    HAL_GPIO_Init(MOTOR_DRIVER_SD_PORT, &GPIO_InitStruct);
    /* 初始化后立即关闭驱动器 */
    s_driver_enable_requested = 0U;
    HAL_GPIO_WritePin(MOTOR_DRIVER_SD_PORT, MOTOR_DRIVER_SD_PIN, MOTOR_DRIVER_SD_DISABLE);
    
    GPIO_InitStruct.Alternate = GPIO_AF1_TIM1; // 使用外设要复用功能
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pin = PWM_UH_PIN | PWM_VH_PIN | PWM_WH_PIN;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(PWM_UH_PORT, &GPIO_InitStruct);

    /* 下桥臂 PB13/PB14/PB15 -> AF1 */
    GPIO_InitStruct.Pin = PWM_UL_PIN | PWM_VL_PIN | PWM_WL_PIN;
    HAL_GPIO_Init(PWM_UL_PORT, &GPIO_InitStruct);

    /* ================================================================
     *  TIM1 三相互补 PWM + CH4 ADC触发 + 死区
     * ================================================================ */
    htim1.Instance = MOTOR_PWM_TIM; // TIM1
    htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED1; /* 中心对齐模式1 */
    htim1.Init.Period = MOTOR_PWM_PERIOD;                    /* 8400, PWM频率=168M/2/8400=10kHz */
    htim1.Init.Prescaler = 0;                                /* 168MHz 不分频  0 */
    htim1.Init.RepetitionCounter = 0;
    HAL_TIM_PWM_Init(&htim1);

    // 管定时器的对外触发信号（TRGO），怎么给其他外设发同步脉冲，此处没用到，只是代码规范层面
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    /* 主模式配置：不使用TRGO触发ADC，由CH4单独触发 */
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

    // PWM通道配置
    TIM_OC_InitTypeDef sConfigOC = {0}; // PWM通道配置结构体
    /* CH1/CH2/CH3 三相PWM通道配置 */
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;     // 输出电平是立刻跳变，还是等当前 PWM 计数周期结束后再跳变,disable 输出跳变和 PWM 周期严格同步
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET; /* 空闲时上桥=低 */
    sConfigOC.OCMode = TIM_OCMODE_PWM1;            // 不用toggle是因为周期不变，翻转模式用于步进电机
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.Pulse = 0;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET; /* 空闲时下桥=低 */
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;    // 互补通道，下桥臂高电平有效
    //    HAL_TIM_OC_Init 整个定时器 + 所有要用的通道，会复位计数器，重新配置时基，适合首次初始化定时器
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1); // 单个通道	不影响定时器，不影响其他通道
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2);
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3);
    //每个通道的硬件会自动生成一路反相、并插入死区的波形，不需要软件手动配置

    // ADC采样通道配置
    sConfigOC.Pulse = MOTOR_PWM_PERIOD - 1U; // 采样点在CNT峰值，即在三角波的谷底，此时所有**下桥臂**处于稳定的**全开状态**，即避开了死区时段，以此触发ADC完成精准电流采样
    HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4);

    /* 死区和刹车配置 */
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE; // 刹车触发后硬件会自动清零主输出使能位 MOE，必须由软件手动重新置位，才能再次开启 PWM 输出
    sBreakDeadTimeConfig.BreakFilter = 0;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;        // 启用定时器的硬件刹车输入引脚
    sBreakDeadTimeConfig.DeadTime = MOTOR_PWM_DEADTIME;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;       // 不开启写保护，所有寄存器随时可软件修改
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSR_DISABLE; // 空闲模式下的输出关闭状态,主输出关断后，所有 PWM 引脚进入高阻态，不再受定时器驱动
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
    HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig);

    /* 初始化占空比清零 */
    PMSM_PWM_Duty_Set(0, 0, 0);
}

// 直接设置三相占空比（FOC/SVPWM 阶段使用）
void PMSM_PWM_Duty_Set(uint16_t u_duty, uint16_t v_duty, uint16_t w_duty)
{
    // 占空比限幅
    if (u_duty > MOTOR_PWM_PERIOD)
        u_duty = MOTOR_PWM_PERIOD;
    if (v_duty > MOTOR_PWM_PERIOD)
        v_duty = MOTOR_PWM_PERIOD;
    if (w_duty > MOTOR_PWM_PERIOD)
        w_duty = MOTOR_PWM_PERIOD;

    // 设置占空比，中心对齐模式下，CNT计数器从0计数到ARR=8400，再从8400计数到0，PWM周期=2*ARR=16800个时钟周期
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, u_duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, v_duty);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, w_duty);
}

// 启动所有PWM输出 : 三相互补通道 + CH4
void PMSM_PWM_Start(void)
{
    //保持开启SD引脚
    s_driver_enable_requested = 0U;
    HAL_GPIO_WritePin(MOTOR_DRIVER_SD_PORT, MOTOR_DRIVER_SD_PIN, MOTOR_DRIVER_SD_DISABLE);
    //上桥臂
    // HAL_TIM_OC_Start------Init 用什么模式，Start 就用什么前缀
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
    //ADC Injected 提供触发采样
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
    //下桥臂
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
    //在PWM输出稳定后，关闭SD引脚
    HAL_GPIO_WritePin(MOTOR_DRIVER_SD_PORT, MOTOR_DRIVER_SD_PIN, MOTOR_DRIVER_SD_ENABLE);
    s_driver_enable_requested = 1U;
}

//关闭所有PWM输出 : 三相互补通道 + CH4
void PMSM_PWM_Stop(void)
{
    //开启SD引脚
    s_driver_enable_requested = 0U;
    HAL_GPIO_WritePin(MOTOR_DRIVER_SD_PORT, MOTOR_DRIVER_SD_PIN, MOTOR_DRIVER_SD_DISABLE);
    //上桥臂
    // HAL_TIM_OC_Start------Init 用什么模式，Start 就用什么前缀
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
    //ADC Injected 提供触发采样
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
    //下桥臂
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3);
}

/* 软件要求驱动使能、但PF10实际被硬件拉低时，判定驱动板已触发保护。 */
uint8_t PMSM_Driver_ShutdownDetected(void)
{
    /*
        条件一：软件明明要求驱动开启
        条件二：实际读取PF10却是低电平
        两个条件同时成立才判断为驱动板硬件保护动作。
    */
    return ((s_driver_enable_requested != 0U) &&
            (HAL_GPIO_ReadPin(MOTOR_DRIVER_SD_PORT, MOTOR_DRIVER_SD_PIN) == MOTOR_DRIVER_SD_DISABLE));
}


