/*
    编码器/SMO得到速度
    → 速度滤波
    → 单位转换成rpm
    → 检查SMO是否有效
    → 选择编码器或SMO速度
    → 检查是否发生来源切换
    → PI无扰处理
    → 作为速度反馈进入速度PI
    → 输出Iq_ref

*/

#include "PMSM.h"

static int8_t loop_step = 0;         // 开环强拖的步数索引
volatile float g_motor_speed = 0.0f; //编码器计算的速度
volatile float g_PMSM_Speed_NowUse_rpm = 0.0f; //当前速度环反馈：编码器或SMO
volatile float g_speed_ref_rpm = 200.0f;//最终目标速度
volatile float g_speed_ref_now_rpm  = 200.0f;//斜坡速度：速度环实际使用时的逐渐变化的目标转速
static uint8_t s_PMSM_speed_NowUse_source = 0U; /* 0:编码器，1:SMO */

TIM_HandleTypeDef htim6;

/* 
    减少切换时速度误差导致电机抖动
    切换速度反馈时重新算PI积分，使下一次PID_Ctrl的首拍输出保持不变。 
 */
static void Motor_SpeedPid_BumplessTransfer(float new_feedback_rpm)
{
    float error = g_speed_pid.SetPoint - new_feedback_rpm;
    float output_hold = g_speed_pid.Output;

    g_speed_pid.Error = error;
    g_speed_pid.Error_Last = error; /* 首拍导数项为0 */
    g_speed_pid.Error_Sum = output_hold
                          - g_speed_pid.Kp * error
                          - g_speed_pid.Ki * error;
    g_speed_pid.Output_raw = output_hold;
}

/* 正反转过零时清除速度PI历史，防止原方向积分继续输出 */
static void Motor_SpeedPid_ClearForReverse(void)
{
    g_speed_pid.Error = 0.0f;
    g_speed_pid.Error_Last = 0.0f;
    g_speed_pid.Error_Sum = 0.0f;
    g_speed_pid.Output_raw = 0.0f;
    g_speed_pid.Output = 0.0f;

    foc_input.Iq_ref = 0.0f;
}

/*   阶梯更新加减速 Ramp：斜坡  
 *      反转时先明确运行到0，再清除旧方向积分，下一拍才能立即建立反方向转矩
 */
static void Motor_SpeedRefRamp_Update(void)
{
    float speed_ref_before = g_speed_ref_now_rpm;
    if (g_speed_ref_now_rpm < g_speed_ref_rpm)
    {
        g_speed_ref_now_rpm = g_speed_ref_now_rpm + SPEED_REF_RAMP_STEP_RPM;
        if (g_speed_ref_now_rpm > g_speed_ref_rpm)
            g_speed_ref_now_rpm = g_speed_ref_rpm;
    }
    else if (g_speed_ref_now_rpm > g_speed_ref_rpm)
    {
        g_speed_ref_now_rpm = g_speed_ref_now_rpm - SPEED_REF_RAMP_STEP_RPM;
        if (g_speed_ref_now_rpm < g_speed_ref_rpm)
            g_speed_ref_now_rpm = g_speed_ref_rpm;
    }
    /*
     * 防止一次斜坡直接从正数跳到负数，或者从负数跳到正数。
     * 必须先经过一次明确的0rpm。
     */
    if (((speed_ref_before > 0.0f) && (g_speed_ref_now_rpm < 0.0f)) ||
        ((speed_ref_before < 0.0f) && (g_speed_ref_now_rpm > 0.0f)))
    {
        g_speed_ref_now_rpm = 0.0f;
    }
    /* 到达0rpm时：清除原方向的速度PI积分 */
    if ((speed_ref_before != 0.0f) && (g_speed_ref_now_rpm == 0.0f))
    {
        Motor_SpeedPid_ClearForReverse();
    }
}


// 带符号速度判断方向：正数正转，负数反转
void Motor_SetSpeedRef(float speed_ref_rpm)
{
    g_speed_ref_rpm = speed_ref_rpm;
}


//由方向判断速度：运行中切换方向，速度环负责制动后反转
//最终目标速度可以立即变成负数，但真正方向必须跟随斜坡速度，不能提前改变

void Motor_SetDirection(uint8_t dir)
{
    float speed_abs = fabsf(g_speed_ref_rpm);
    g_speed_ref_rpm = (dir == CCW) ? -speed_abs : speed_abs;
    // foc_input.Dir = (dir == CCW) ? CCW : CW;//根据输入方向设置方向
    // g_speed_ref_rpm = (foc_input.Dir == CW) ? speed_abs : -speed_abs;
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
            Motor_State control_state = Motor_GetState();

            /* 闭环中对目标速度限斜率，反转时先减速过零再反向加速。 */
            if ((control_state == motor_state_encoder_run) || (control_state == motor_state_smo_run))
                Motor_SpeedRefRamp_Update();
            else
                //开环状态：速度PID没有运行,电机速度由角度递增速度决定,开环状态下匀速运行。
                g_speed_ref_now_rpm  = g_speed_ref_rpm;

            /* 允许调试器或通信层直接写入带符号速度给定
             * g_speed_ref_rpm是目标速度，g_speed_ref_now_rpm当前速度 
             * 根据当前速度来判断实际方向，如果使用目标速度判断实际方向反方向的PI会沿用正方向的PI值，导致输出数据突然变大或停机
             */
            if (g_speed_ref_now_rpm > 0.0f)
                foc_input.Dir = CW;
            else if (g_speed_ref_now_rpm < 0.0f)
                foc_input.Dir = CCW;

            /* 编码器或SMO闭环状态下运行速度外环，因为角度开环运行的角度是无法改变的，速度环调节会改变角度。 */
            float smo_speed_snapshot;//本次速度环使用的SMO速度快照
            uint8_t smo_input_valid;//SMO输入数据是否正常
            uint8_t smo_angle_valid;//SMO当前角度、速度是否可信
            uint8_t new_speed_source;//本次准备使用哪种速度反馈,   1无感，0有感.
            uint8_t speed_source_changed;//速度反馈来源是否刚刚发生切换

            control_state = Motor_GetState();
            smo_speed_snapshot = SMO_Output.speed_rpm;
            smo_input_valid = SMO_Output.input_valid;
            smo_angle_valid = SMO_Output.angle_valid;


            /*
             * 编码器速度始终保留用于监测；仅在健康的SMO无感态切换速度环反馈。
             * 若SMO刚失效，本拍也优先用编码器，避免速度PI因反馈清零冲击。
             */
            if ((control_state == motor_state_smo_run) && (smo_input_valid != 0U) &&
                (smo_angle_valid != 0U) && !isnan(smo_speed_snapshot) && !isinf(smo_speed_snapshot))
            {//smo
                g_PMSM_Speed_NowUse_rpm = smo_speed_snapshot;
                new_speed_source = 1U;
            }
            else
            {//编码器
                g_PMSM_Speed_NowUse_rpm = g_motor_speed;
                new_speed_source = 0U;
            }
            speed_source_changed = (new_speed_source != s_PMSM_speed_NowUse_source) ? 1U : 0U;
            s_PMSM_speed_NowUse_source = new_speed_source;//切换状态
            
            if ((control_state == motor_state_encoder_run) || (control_state == motor_state_smo_run))
            {
                g_speed_pid.SetPoint = g_speed_ref_now_rpm ;//把当前经过斜坡处理后的目标转速，设置为速度PI控制器的目标值
                if (speed_source_changed != 0U)
                {//发送切换控制，命令原来的速度给PI控制器，重新计算积分，使下一拍输出尽量保持不变。
                    Motor_SpeedPid_BumplessTransfer(g_PMSM_Speed_NowUse_rpm);//当前实际采用的速度
                }

                /* 表贴式PMSM先使用Id=0 */
                foc_input.Id_ref = 0.0f;

                /* 速度PI输出作为Iq_ref */
                foc_input.Iq_ref =
                    PID_Ctrl(&g_speed_pid, g_PMSM_Speed_NowUse_rpm);
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


