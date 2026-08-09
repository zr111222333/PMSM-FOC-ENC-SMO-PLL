#include "PMSM.h"

static int8_t loop_step = 0; // 开环强拖的步数索引
volatile float g_motor_speed = 0.0f; //编码器计算的速度
volatile float g_speed_ref_rpm = 200.0f;
static float s_speed_pid_ref_rpm = 200.0f;

TIM_HandleTypeDef htim6;

static void Motor_SpeedRefRamp_Update(void)
{
    if (s_speed_pid_ref_rpm < g_speed_ref_rpm)
    {
        s_speed_pid_ref_rpm += SPEED_REF_RAMP_STEP_RPM;
        if (s_speed_pid_ref_rpm > g_speed_ref_rpm)
            s_speed_pid_ref_rpm = g_speed_ref_rpm;
    }
    else if (s_speed_pid_ref_rpm > g_speed_ref_rpm)
    {
        s_speed_pid_ref_rpm -= SPEED_REF_RAMP_STEP_RPM;
        if (s_speed_pid_ref_rpm < g_speed_ref_rpm)
            s_speed_pid_ref_rpm = g_speed_ref_rpm;
    }
}

void Motor_SetSpeedRef(float speed_ref_rpm)
{
    g_speed_ref_rpm = speed_ref_rpm;
    if (speed_ref_rpm > 0.0f)
        foc_input.Dir = CW;
    else if (speed_ref_rpm < 0.0f)
        foc_input.Dir = CCW;
}

void Motor_SetDirection(uint8_t dir)
{
    float speed_abs = fabsf(g_speed_ref_rpm);

    foc_input.Dir = (dir == CCW) ? CCW : CW;
    g_speed_ref_rpm = (foc_input.Dir == CW) ? speed_abs : -speed_abs;
}

// 以前六步换相阶段使用的初始化函数
void PMSM_Init(void)
{
    Motor_Six_Step_Stop();
    Encoder_Start(); 
}

/**
 * @brief  单步六步换相PWM输出
 * @param  step: 换相步骤 0~5
 * @param  duty: PWM占空比原始值（建议初期不超过500，低电压验证）
 * @note   导通逻辑：
 *         0: U+ V-
 *         1: U+ W-
 *         2: V+ W-
 *         3: V+ U-
 *         4: W+ U-
 *         5: W+ V-
 */
void Motor_Six_Step_SetPhase(uint8_t step, uint16_t duty)
{
    switch (step)
    {

    case 0: /*U+ V- (U相上桥PWM，V相下桥导通，W相悬空)*/
        BLDC_PWM_Duty_Set(duty, 0, 0);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
        break;
    case 1: // U+ W-
        BLDC_PWM_Duty_Set(duty, 0, 0);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_SET);
        break;
    case 2: // V+ W-
        BLDC_PWM_Duty_Set(0, duty, 0);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_SET);
        break;
    case 3: // V+ U-
        BLDC_PWM_Duty_Set(0, duty, 0);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
        break;
    case 4: // W+ U-
        BLDC_PWM_Duty_Set(0, 0, duty);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
        break;
    case 5: // W+ V-
        BLDC_PWM_Duty_Set(0, 0, duty);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
        break;
    default:
        BLDC_PWM_Duty_Set(0, 0, 0);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
        break;
    }
}

/**
 * @brief  开环强拖：连续六步换相运行
 * @param  dir: 旋转方向
 * @param  duty: 占空比原始值（低占空比验证，建议<800）
 * @note   需在定时中断或周期任务中调用，调用频率决定换相速度
 */
void Motor_Six_Step_Run(uint8_t dir, uint16_t duty)
{
    if (dir == CW)
    {
        Motor_Six_Step_SetPhase(loop_step, duty);
        loop_step++;
        if (loop_step >= 6)
            loop_step = 0;
    }
    else if (dir == CCW)
    {
        Motor_Six_Step_SetPhase(loop_step, duty);
        loop_step--;
        if (loop_step < 0)
            loop_step = 5;
    }
}

/**
 * @brief  开环强拖的六步换相停止，所有桥臂关闭
 */
void Motor_Six_Step_Stop(void)
{
    BLDC_PWM_Duty_Set(0, 0, 0);
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_UL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_VL_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PWM_UL_PORT, PWM_WL_PIN, GPIO_PIN_RESET);
    loop_step = 0;
}

// TIM6: 定时计算速度--->周期： 因为f_PWM=f_Iadc=10KHz,所以10kHz 电流环 ÷ 10 = 1kHz 速度环
void SpeedLoop_TIM6_Init(void)
{
    __HAL_RCC_TIM6_CLK_ENABLE();
    htim6.Instance = TIM6;
    htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    htim6.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim6.Init.Period = 1000 - 1; // 1MHz / 1000 = 1kHz = 1ms
    htim6.Init.Prescaler = 83;    // 84MHz / 84 = 1MHz
    htim6.Init.RepetitionCounter = 0;
    HAL_TIM_Base_Init(&htim6);

    HAL_NVIC_SetPriority(TIM6_DAC_IRQn, 2, 1); // 优先级低于ADC电流环
    HAL_NVIC_EnableIRQ(TIM6_DAC_IRQn);
}

void TIM6_DAC_IRQHandler(void)
{
    HAL_TIM_IRQHandler(&htim6);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{

    static uint8_t speed_loop_cnt = 0;
    if (htim->Instance == TIM6)
    {
        /* 获取编码器位置在TIM1-ADC中断，因为高频角度偏差小速度准确 */
        Encoder_CNT_Update_MultiTurn();
        
        speed_loop_cnt++;
        /* 每5ms计算速度并运行一次速度环 */
        if (speed_loop_cnt >= 5U)
        {
            speed_loop_cnt = 0U;
            /* 每个周期只计算一次机械转速，单位rpm */
            g_motor_speed = Encoder_Speed() / (2 * PI) * 60.0f; // 转速单位：r/min

            /* 闭环中对目标速度限斜率，反转时先减速过零再反向加速。 */
            if (Motor_GetState() == motor_state_encoder_run)
                Motor_SpeedRefRamp_Update();
            else
                s_speed_pid_ref_rpm = g_speed_ref_rpm;

            /* 允许调试器或通信层直接写入带符号速度给定。 */
            if (g_speed_ref_rpm > 0.0f)
                foc_input.Dir = CW;
            else if (g_speed_ref_rpm < 0.0f)
                foc_input.Dir = CCW;

            /* 编码器闭环状态下运行速度外环，因为角度开环运行的角度是无法改变的，速度环调节会改变角度。 */
            if (Motor_GetState() == motor_state_encoder_run)
            {
                g_speed_pid.SetPoint = s_speed_pid_ref_rpm;

                /* 表贴式PMSM先使用Id=0 */
                foc_input.Id_ref = 0.0f;

                /* 速度PI输出作为Iq_ref */
                foc_input.Iq_ref =
                    PID_Ctrl(&g_speed_pid, g_motor_speed);
            }
        }
        // 过流、过压、欠压等保护检测
        Motor_Protect_Check();
    }
}

/*   TIM6启动：速度环  */
void SpeedLoop_TIM6_Start(void)
{
    HAL_TIM_Base_Start_IT(&htim6);
}


