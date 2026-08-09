#include "Encoder.h"

/*    */
TIM_HandleTypeDef htim3;

volatile float g_encoder_elec_angle = 0.0f;//变量名代替函数读取角度

/* 多圈位置相关变量 */
static int32_t cnt_last = 0;   // 上一次的CNT值
static int32_t cnt_total = 0;  // 累计的多圈绝对位置
static int32_t cnt_offset = 0;          /* 编码器Z相对应的机械计数偏置 */
static float angle_last = 0.0f;
static float elec_angle_offset = 0.0f; /* 编码器原始CNT零点到转子d轴之间的电角度偏移 */
static uint8_t z_phase_found = 0;     /* 定义一个标志位，记录是否已经找到过零点 */

/**
 * @brief  编码器GPIO和时钟初始化
 */
void Encoder_Init(void)
{
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();
    __HAL_RCC_TIM9_CLK_ENABLE(); // Z
    __HAL_RCC_TIM3_CLK_ENABLE(); // AB

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    /*  A B */
    GPIO_InitStruct.Pin = Encoder_A_Pin | Encoder_B_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP; /*  当引脚是给片上定时器外设做硬件输入时，必须配置为复用功能模式 **，而不是普通 GPIO 输入模式。  */
    GPIO_InitStruct.Alternate = Encoder_AF;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW; // 编码器接口是定时器的输入通道
    HAL_GPIO_Init(Encoder_A_Port, &GPIO_InitStruct);
    /*  Z */
    GPIO_InitStruct.Pin = Encoder_Z_Pin;
    // GPIO_InitStruct.Alternate=;      //z相脉冲是编码器硬件自身产生的，使用 GPIO 引脚自带的 EXTI 中断能力回零
    GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN; /* 下拉，默认低电平 */
    HAL_GPIO_Init(Encoder_Z_Port, &GPIO_InitStruct);

    htim3.Instance = Encoder_TIM;
    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1; // 滤波器时钟最高 → 滤波时间最短
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = Encoder_Period; // 编码器计数值4000-1
    htim3.Init.Prescaler = 0;
    htim3.Init.RepetitionCounter = 0;

    TIM_Encoder_InitTypeDef sConfig = {0};
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Filter = 0; // 滤波值范围0个时钟周期
    sConfig.IC1Polarity = TIM_ENCODERINPUTPOLARITY_RISING;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI; // 手册15.4.7 CC1S
    sConfig.IC2Filter = 0;
    sConfig.IC2Polarity = TIM_ENCODERINPUTPOLARITY_RISING;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    HAL_TIM_Encoder_Init(&htim3, &sConfig);

    /* 定时器中断等间隔采样位置和速度 */
    HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(TIM3_IRQn);
    /* Z 相 EXTI 中断 */
    HAL_NVIC_SetPriority(EXTI9_5_IRQn, 2, 0);
    HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

    //  __HAL_TIM_ENABLE_IT(&htim3, TIM_IT_UPDATE);
}

/*  中断处理  */
void TIM3_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim3);
}

void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(Encoder_Z_Pin);
}

/* Z 相中断回调：触发时标记零位，可在此做计数器清零，消除累计误差 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == Encoder_Z_Pin)
    {
        if (z_phase_found == 0) /* 只有开机第一次找机械零点时执行 */
        {
            // 记录此时的CNT作为零点偏置
            cnt_offset = __HAL_TIM_GetCounter(&htim3);
            // 同步清零多圈累计位置
            cnt_total = 0;         // 绝对位置
            cnt_last = cnt_offset; // 上次位置
            angle_last = 0.0f;     // 同步刷新速度计算的历史角度，防止速度计算突变

            /* 3. 标记为已找到，后续运转中不再干预角度 */
            z_phase_found = 1;

            /* 4. 关闭 Z 相外部中断，彻底杜绝运转中的角度突变 */
            HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
        }
    }
}

/*  启动编码器   */
void Encoder_Start(void)
{
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_1); // A
    HAL_TIM_IC_Start(&htim3, TIM_CHANNEL_2); // B
    // 初始化上次计数值
    cnt_last = (int32_t)__HAL_TIM_GetCounter(&htim3);
}

/*  停止编码器   */
void Encoder_Stop(void)
{
    HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_1); // A
    HAL_TIM_IC_Stop(&htim3, TIM_CHANNEL_2); // B
}

/*  外部接口：获取编码器计数值(带符号)   */
int32_t Encoder_Get_Cnt(void)
{
    return (int32_t)__HAL_TIM_GetCounter(&htim3);
}

/*    获取编码器CNT值   */
void Encoder_CNT_Update_MultiTurn(void)
{
    int32_t cnt_now = Encoder_Get_Cnt();
    int32_t cnt_delta = cnt_now - cnt_last; // cnt变化值
    /*  编码器多圈累加---溢出差分法  */
    if (cnt_delta > 2000) // 差值过大(>2000)，说明是反转溢出(从0越界到了3999)
        cnt_delta = cnt_delta - 4000;
    else if (cnt_delta < -2000) // 差值过大(<-2000)，说明是正转溢出(从3999越界到了0)
        cnt_delta = 4000 + cnt_delta;
    cnt_total = cnt_total + cnt_delta; // 累加多圈得编码器绝对位置
    cnt_last = cnt_now;                // 保存本次值供下次使用
}

/*    编码器获取机械角度   */
float Encoder_Angle(void)
{
    int32_t cnt_now = Encoder_Get_Cnt() - cnt_offset;
    float angle = (float)cnt_now / 4000.0f * 2.0f * PI; // 4000为编码器计数值，2*PI为一圈的弧度
    /* 角度归一化到0~2π */
    if (angle < 0)
        angle = 2.0f * PI + angle;
    else if (angle >= 2.0f * PI)
        angle = angle - 2.0f * PI;
    return angle;
}

/*
 * @brief 获取编码器电角度，或在转子对齐完成后校准电角度偏置
 *
 * @param calibrate_enable
 *        ENCODER_ELEC_ANGLE_READ：
 *            正常读取FOC使用的电角度
 *
 *        ENCODER_ELEC_ANGLE_CALIBRATE：
 *            转子对齐完成后，记录编码器与转子d轴之间的电角度偏置
 *
 * @param align_elec_angle
 *        对齐阶段施加的固定电角度。
 *        正常读取时该参数传0.0f即可。
 *
 * @retval 补偿偏置后的电角度，范围0~2π
 */
float Encoder_ElecAngle(uint8_t calibrate_enable)
{
    /*
    * FOC电角度必须使用原始CNT。
    * 不使用Encoder_Angle()，避免受到Z相机械零点cnt_offset影响。
    */
    // float elec_angle = Encoder_Angle() * MOTOR_POLE_PAIRS; // 电角度 = 机械角度 * 极对数
    float mech_angle_raw =(float)Encoder_Get_Cnt() / (float)(Encoder_Period + 1U) * 2.0f * PI;
    /* 原始编码器机械角度转换为电角度 */
    float elec_angle = fmodf( mech_angle_raw * 4.0f, 2.0f * PI );             // fmodf：elec_angle对2PI其余
    /* 角度归一化到0~2π */
    if (elec_angle < 0.0f)
        elec_angle = 2.0f * PI + elec_angle;
    /* 转子对齐完成时，记录转子d轴对应的编码器电角度 */
    if (calibrate_enable == Encoder_elec_angel_calibrate)
    {
        elec_angle_offset = elec_angle;
    }
    /* 得到以转子d轴为零点的FOC电角度 */
    elec_angle = elec_angle -elec_angle_offset;
    /* 电角度归一化到0~2π */
    if (elec_angle < 0.0f)
    {
        elec_angle += 2.0f * PI;
    }
    else if (elec_angle >= 2.0f * PI)
    {
        elec_angle -= 2.0f * PI;
    }

    g_encoder_elec_angle = elec_angle;
    return g_encoder_elec_angle;
}

/*    编码器计算速度：TIM6调用   */
float Encoder_Speed(void)
{
    static float Encoder_Speed_Filter_Now = 0.0f;
    static float Encoder_Speed_Filter_Last = 0.0f;
    static uint8_t First_Speed_Filter = 1; /* 首次计算标志，防止初值为0导致起步滤波拉扯 */

    float angle_now = Encoder_Angle();
    float angle_delta = angle_now - angle_last;

    /* 处理单圈角度过零跳变 */
    if (angle_delta > PI) // 逆转过零
        angle_delta = angle_delta - 2.0f * PI;
    else if (angle_delta < -PI) // 正转过零
        angle_delta = angle_delta + 2.0f * PI;
    
    float Encoder_Speed_Raw = angle_delta / 0.005f; // 5ms计算一次速度，1ms太快虽然精度高但波动太大，单位：rad/s

    /* 开机第一次直接赋值，消除0初值滞后 */
    if (First_Speed_Filter == 1)
    {
        Encoder_Speed_Filter_Now = Encoder_Speed_Raw;
        First_Speed_Filter = 0;
    }
    else /* 一阶低通滤波: y[n] = α * x[n] + (1-α) * y[n-1] */
    {
        Encoder_Speed_Filter_Now = Encoder_Speed_Raw * Speed_LPF_Alpha + Encoder_Speed_Filter_Last * (1.0f - Speed_LPF_Alpha);
    }

    Encoder_Speed_Filter_Last = Encoder_Speed_Filter_Now;
    angle_last = angle_now;
    return Encoder_Speed_Filter_Now;
}
