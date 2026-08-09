#include "Motor_State.h"

/* 当前电机状态，主循环与ADC中断都会访问 */
volatile Motor_State g_motor_state = motor_state_idle;
/*  
 *  CAN 只是增加了一种“下达启动、速度和模式命令”的方式，并没有改变电机原来的控制原理。
 *   当前代码默认目标模式本来就是 SMO；无论通过按键还是 CAN 启动，都会先经过开环和编码器控制，最后满足安全条件才切换到 SMO。
 */
volatile Motor_ControlMode g_motor_control_mode = motor_control_smo;//看通信初始设置的目标控制模式：期望控制模式
volatile float g_encoder_smo_angle_error = 0.0f;;//保存编码器角度和SMO角度之间的最短角度误差
volatile float g_foc_elec_angle = 0.0f;         //保存当前真正输入FOC的电角度

static volatile uint32_t state_tick = 0;/*  进入新状态的时间    */
static volatile uint8_t state_entry = 1;/*  标记新状态第一次进入    */
static volatile Motor_State s_previous_state = motor_state_idle;/*   初始值设为空闲状态   */
static float open_loop_theta = 0.0f;/* FOC开环电角度 */
static Motor_State s_fast_state = motor_state_idle;//10kHz快速控制上一拍使用的状态,用来是否发生了角度来源切换
static float s_angle_sync_offset = 0.0f;//切换时角度偏移角
static uint32_t s_smo_stable_count = 0U;


/* 角度归一化到0~2pi */
static float Motor_WrapAngle(float angle)
{
    while (angle >= 2.0f * PI)
    {
        angle -= 2.0f * PI;
    }
    while (angle < 0.0f)
    {
        angle += 2.0f * PI;
    }
    return angle;
}

/* 返回angle_a-angle_b的最短角差，范围[-pi, pi]，角度差超过半圈的情况下不太可能，因为电机速度达不到这么快。。 */
static float Motor_AngleDifference(float angle_a, float angle_b)
{
    float error = angle_a - angle_b;
    while (error > PI)
    {
        error -= 2.0f * PI;
    }
    while (error < -PI)
    {
        error += 2.0f * PI;
    }
    return error;
}

//判断当前是否需要执行FOC
static uint8_t Motor_StateUsesFoc(Motor_State state)
{
    return ((state == motor_state_align) ||
            (state == motor_state_open_loop) ||
            (state == motor_state_encoder_run) ||
            (state == motor_state_smo_run)) ? 1U : 0U;
}

//给SMO接管FOC之前增加统一的安全检查,判断SMO是否有效值
static uint8_t Motor_SMO_IsHealthy(void)
{
    float pll_theta = SMO_Output.PLL_theta;
    float speed_rpm = SMO_Output.speed_rpm;

    if ((SMO_Output.input_valid == 0U) || (SMO_Output.angle_valid == 0U) ||
        isnan(pll_theta) || isinf(pll_theta) ||
        isnan(speed_rpm) || isinf(speed_rpm))
    {
        return 0U;
    }
    return 1U;
}

//判断SMO和编码器相互接管的时候方向是否一致,防止方向不一致时接管导致电机跳变
static uint8_t Motor_SpeedDirectionMatches(float encoder_speed_rpm, float smo_speed_rpm)
{
    return ((encoder_speed_rpm * smo_speed_rpm) > 0.0f) ? 1U : 0U;
}


/* 判断当前是不是第一次进入某个状态，同时防止1ms状态机与10kHz中断,故障保护同时修改状态时产生竞争问题 */
static uint8_t Motor_TakeStateEntry(Motor_State expected_state)
{
    uint32_t primask = __get_PRIMASK();
    uint8_t take_entry = 0U;

    __disable_irq();
    if ((g_motor_state == expected_state) && (state_entry != 0U))
    {
        state_entry = 0U;
        take_entry = 1U;//状态无误进入
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return take_entry;
}

//状态机初始化
void Motor_StateInit(void)
{
    g_motor_state = motor_state_idle;//电机实际初始状态为空闲状态(实际值)
    g_motor_control_mode = motor_control_smo;//CAN控制模式默认使用SMO无感控制(期望值)
    state_entry = 1U;
    state_tick = HAL_GetTick();
    s_previous_state = motor_state_idle;
    open_loop_theta = 0.0f;
    s_fast_state = motor_state_idle;
    s_angle_sync_offset = 0.0f;
    s_smo_stable_count = 0U;
    g_foc_elec_angle = 0.0f;
    g_encoder_smo_angle_error = 0.0f;
}

/*
 * @brief 只负责切换状态，不直接执行复杂控制
  * 1. IDLE：全部关闭；
 * 2. CURRENT_OFFSET：只启动CH4；
 * 3. ALIGN：启动三相PWM和CH4；
 * 4. OPEN_LOOP：继续运行PWM；
 * 5. ENCODER_RUN/SMO_RUN：分别使用编码器或SMO闭环；
 * 6. FAULT：全部关闭。
 */
void Motor_SetState(Motor_State state)
{
    uint32_t primask = __get_PRIMASK();//记录当前CPU是否已经关闭中断
    Motor_State old_state;

    __disable_irq();//防止状态只更新了一半，中断就进来运行
    old_state = g_motor_state;
    /* 故障状态锁存，只有重新执行Motor_StateInit后才能离开。 */
    if ((state == old_state) ||
        ((old_state == motor_state_fault) && (state != motor_state_fault)) ||
        ((g_protect.Fault != Fault_NONE) && (state != motor_state_fault)))
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }
    s_previous_state = old_state;
    state_tick = HAL_GetTick(); /*  记录进入新状态的时间    */
    state_entry = 1;            /*  标记新状态第一次进入    */
    __DMB();
    g_motor_state = state;      /* 最后提交状态，快速环不会看到半更新元数据 */
    if (primask == 0U)
    {
        __enable_irq();
    }
}

/**
 * @brief 获取当前电机状态
 */
Motor_State Motor_GetState(void)
{
    return g_motor_state;
}

/**
 * @brief CAN设置期望电机控制模式
 * @param mode: 目标控制模式
 * @return: 1表示设置成功，0表示设置失败
 */
uint8_t Motor_SetControlMode(Motor_ControlMode mode)
{
    Motor_State state;
    if ((mode > motor_control_smo) || (g_protect.Fault != Fault_NONE))//如果参数超出范围，或者电机处于故障状态，则返回失败
        return 0U;
    g_motor_control_mode = mode;
    state = Motor_GetState();
    if ((mode == motor_control_open_loop) &&
        ((state == motor_state_encoder_run) || (state == motor_state_smo_run)))//如果当前状态是编码器闭环或者SMO闭环，则切换到FOC开环
        Motor_SetState(motor_state_open_loop);
    else if ((mode == motor_control_encoder) && (state == motor_state_smo_run))//如果当前状态是SMO闭环，则切换到编码器闭环
        Motor_SetState(motor_state_encoder_run);
    return 1U;
}

/**
 * @brief 获取当前CAN电机控制模式
 * @return: 当前控制模式
 */
Motor_ControlMode Motor_GetControlMode(void)
{
    return g_motor_control_mode;
}




/*
 * @brief 低频控制任务 (主循环约1kHz调度, TIM6触发):状态进入、等待时间和状态转换,负责决定当前模式
 */
void Motor_StateTask_1ms(void)  //Motor_SpeedLoop_Task
{

    if (g_protect.Fault != Fault_NONE)//进入故障状态
        Motor_Identify_Abort(motor_identify_fail_protection);//参数辨识中止
    Motor_Identify_ReportOnce();   //可能会和串口打印波形冲突，标记一下

    switch(g_motor_state)
    {
        /*   空闲状态   */
        case motor_state_idle:
        {
            /* 进入停机状态时只执行一次 */
            if (Motor_TakeStateEntry(motor_state_idle) != 0U)
            {
                /* 关闭PWM和驱动 */
                PMSM_PWM_Duty_Set(0,0,0);
                PMSM_PWM_Stop();
                /* 清除电流给定 */
                foc_input.Id_ref = 0.0f;
                foc_input.Iq_ref = 0.0f;
                /* 只清除PID历史值 */
                PID_Init(&g_current_pid_Id,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
                PID_Init(&g_current_pid_Iq,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
            }
            break;
        }
        /*   零偏电流校准   */
        case motor_state_current_offset:
            if (Motor_TakeStateEntry(motor_state_current_offset) != 0U)
            {
                /* 校准时关闭功率驱动 */
                PMSM_PWM_Stop();
                /* 只启动CH4，为注入ADC检测电流提供触发源 */
                HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
                state_tick = HAL_GetTick();
            }
            /* 等待停机电流完全衰减 */
            if(HAL_GetTick()-state_tick >= 200)
            {
                Motor_ADC_Get_Current_Offset(); 
                HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_4);
                /* 零偏校准完成，进入转子对齐 */
                Motor_SetState(motor_state_align);
            }
            break;
        /*   转子对齐状态   */
        case motor_state_align:
        {
            if (Motor_TakeStateEntry(motor_state_align) != 0U)
            {
                /* 清除之前的电流环状态 */
                PID_Init(&g_current_pid_Id,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
                PID_Init(&g_current_pid_Iq,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
                /* 新一轮启动清除SMO历史，首拍会自动丢弃停机前旧电压。 */
                SMO_Init();
                if ((g_motor_state == motor_state_align) && (g_protect.Fault == Fault_NONE))
                {
                    /* 固定电角度0，施加Id完成转子对齐 */
                    foc_input.Id_ref = Align_Id;
                    foc_input.Iq_ref = 0.0f;
                    /* 清除旧PWM，先输出零电压 */
                    PMSM_PWM_Duty_Set(0,0,0);
                    /* 启动PWM输出 */
                    PMSM_PWM_Start();
                    state_tick = HAL_GetTick();
                }
                else
                {
                    foc_input.Id_ref = 0.0f;
                    foc_input.Iq_ref = 0.0f;
                }
            }
            /* 等待转子对齐完成 */
            if ((g_motor_state == motor_state_align) && (HAL_GetTick() - state_tick >= Align_Time_ms))
            {
                /* 保存当前位置对应的编码器电角度偏置 */
                Encoder_ElecAngle(Encoder_elec_angel_calibrate);
                foc_input.Id_ref = 0;
                /* 清除之前的dq电流环状态 */
                PID_Init(&g_current_pid_Id,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
                PID_Init(&g_current_pid_Iq,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
                /* 转子对齐完成重置角度 */
                open_loop_theta = 0.0f;
                // 进入参数辨识状态
                if (Motor_Identify_WasAttempted() == 0U)//如果没有尝试过参数辨识，就进入参数辨识状态
                    Motor_SetState(motor_state_identify);
                else
                    Motor_SetState(motor_state_open_loop);/* 转子对齐完成，进入FOC开环 */
            }
            break;
        }
        /*   参数辨识状态   */
        case motor_state_identify:
        {
            if (Motor_TakeStateEntry(motor_state_identify) != 0U)
            {
                PMSM_PWM_Duty_Set(MOTOR_PWM_PERIOD / 2U,
                                   MOTOR_PWM_PERIOD / 2U,
                                   MOTOR_PWM_PERIOD / 2U);
                Motor_Identify_Start();
            }
            if ((g_motor_state == motor_state_identify) && (Motor_Identify_IsFinished() != 0U))
                Motor_SetState(motor_state_align);
            break;
        }
        //FOC开环状态
        case motor_state_open_loop:
        {
            if (Motor_TakeStateEntry(motor_state_open_loop) != 0U)
            {
                foc_input.Id_ref = 0;
                foc_input.Dir = (g_speed_ref_rpm < 0.0f) ? CCW : CW;//速度判断方向
                foc_input.Iq_ref = (foc_input.Dir == CW) ? Open_Loop_Iq : -Open_Loop_Iq;
                PID_Init(&g_current_pid_Id,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
                PID_Init(&g_current_pid_Iq,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
                PID_Init(&g_speed_pid,S_Kp, S_Ki, S_Kd, S_Kb,S_OutMax, S_OutMin);
            }
            /*
             * 开环约119rpm机械转速，运行满指定时间且编码器确认已跟转后，
             * 自动交给编码器低速闭环；按键仍可提前切换用于调试。
             */
            if ((HAL_GetTick() - state_tick >= OPEN_LOOP_RUN_TIME_MS) && 
                (g_motor_control_mode != motor_control_open_loop) &&  //如果CAN期望模式不是开环模式，就切换到闭环模式，避免开环自动切换到闭环模式
                (((foc_input.Dir == CW) && (g_motor_speed >= OPEN_LOOP_MIN_ENCODER_SPEED_RPM)) ||
                 ((foc_input.Dir == CCW) && (g_motor_speed <= -OPEN_LOOP_MIN_ENCODER_SPEED_RPM))))
            {
                Motor_SetState(motor_state_encoder_run);
            }
            break;
        }
        //编码器有感状态
        case motor_state_encoder_run:
        {
             if (Motor_TakeStateEntry(motor_state_encoder_run) != 0U)
            {
                foc_input.Id_ref = 0.0f;
                /*
                 * 开环首次状态变化进入编码器闭环时初始化速度PI；
                 * SMO异常回退时保留PI积分和Iq，避免转矩突变。
                 */
                if (s_previous_state != motor_state_smo_run)
                {
                    foc_input.Iq_ref = 0.0f;
                    /* 初步测试可以继续使用开环阶段的Iq_ref
                        主要就是这句注释，在电流环时使用 IQ 导致速度变化，电流不稳定，KPKi 调节无法使用。 */
                    // foc_input.Iq_ref = Open_Loop_Iq;

                    /* 进入速度闭环前清空速度PI历史值 */
                    PID_Init(&g_speed_pid,S_Kp, S_Ki, S_Kd, S_Kb,S_OutMax, S_OutMin);
                    g_speed_pid.SetPoint = g_speed_ref_rpm;
                }
            }
            break;
        }
        //SMO无感状态
        case motor_state_smo_run:
        {
            if (Motor_TakeStateEntry(motor_state_smo_run) != 0U)
            {
                /* 从刚才检查到现在，状态有没有被中断改变 */
                if ((g_motor_state == motor_state_smo_run) && (g_protect.Fault == Fault_NONE))
                {
                    foc_input.Id_ref = 0.0f;//确保不会残留之前状态的Id
                }
            }
            /*
             * 300rpm回差退出；SMO无效、双通道方向相反或角差过大均退回编码器。
             * 快速环还会对SMO无效和严重角差执行更快的同周期回退。
             */
            if ((Motor_SMO_IsHealthy() == 0U) ||
                // (fabsf(SMO_Output.speed_rpm) < SMO_SWITCH_DOWN_RPM) ||
                (fabsf(g_motor_speed) < SMO_SWITCH_DOWN_RPM) ||
                (fabsf(g_encoder_smo_angle_error) > SMO_RUN_ANGLE_ERROR_MAX_RAD) ||
                (Motor_SpeedDirectionMatches(g_motor_speed, SMO_Output.speed_rpm) == 0U))
            {
                Motor_SetState(motor_state_encoder_run);
            }
            break;
        }
        case motor_state_fault:
        {
            if (Motor_TakeStateEntry(motor_state_fault) != 0U)
            {
                foc_input.Id_ref = 0.0f;
                foc_input.Iq_ref = 0.0f;
                PMSM_PWM_Duty_Set(0, 0, 0);
                PMSM_PWM_Stop();
            }
            break;
        }
        default:
        {
            Motor_SetState(motor_state_fault);
            break;
        }
    }
}

/*
 * @brief 10 kHz快速控制:选择辨识或FOC角度源并在同周期做SMO切入/回退判定
 */
void Motor_FastControlISR(void)
{
    Motor_State state = g_motor_state;//本次10kHz中断使用的状态
    float encoder_theta;              //编码器电角度
    float source_theta;               //交给FOC的角度
    //如果保护已经触发，这一拍不能再继续计算并更新PWM
    if (g_protect.Fault != Fault_NONE)
    {
        return;
    }

    if (state == motor_state_identify)
    {
        Motor_Identify_FastTask();
        s_fast_state = state;
        s_angle_sync_offset = 0.0f;
        return;
    }

    //非FOC状态直接退出,像停机、零偏校准、故障状态都不应该继续执行FOC
    if (Motor_StateUsesFoc(state) == 0U)
    {
        s_fast_state = state;
        s_angle_sync_offset = 0.0f;
        return;
    }

    /* 所有FOC状态都读取编码器；SMO在Foc_Run_Core内部持续运行。 */
    encoder_theta = Encoder_ElecAngle(Encoder_elec_angel_read);
    g_encoder_smo_angle_error = Motor_AngleDifference(encoder_theta, SMO_Output.PLL_theta);

    /* 在10kHz边界逐拍验证，确保切入条件真正连续稳定200ms。 */
    if (state == motor_state_encoder_run)// 第一层：当前是否处于编码器状态
    {
        // 第二层：SMO是否稳定可靠
        if (((g_motor_control_mode == motor_control_smo) && //CAN期望模式是SMO
             fabsf(g_motor_speed) >= SMO_SWITCH_UP_RPM) &&  //电机实际速度＞可以无感切换时的速度
            (Motor_SMO_IsHealthy() != 0U) &&    //SMO是有效值
            (fabsf(g_encoder_smo_angle_error) <= SMO_SWITCH_ANGLE_ERROR_MAX_RAD) &&//角度偏差低
            (Motor_SpeedDirectionMatches(g_motor_speed, SMO_Output.speed_rpm) != 0U) &&//有无感方向一致
            (fabsf(g_motor_speed - SMO_Output.speed_rpm) <= SMO_SWITCH_SPEED_ERROR_MAX_RPM))//速度偏差小
        {//连续稳定200ms才能切换
            if (s_smo_stable_count < SMO_SWITCH_STABLE_COUNT)
            {
                s_smo_stable_count++;// 连续稳定次数累加
            }
            if (s_smo_stable_count >= SMO_SWITCH_STABLE_COUNT)
            {
                s_smo_stable_count = 0U;
                Motor_SetState(motor_state_smo_run);
                state = motor_state_smo_run;
            }
        }
        else// 在编码器状态，但SMO条件有一项不满足
        {
            s_smo_stable_count = 0U;
        }
    }
    else// 已经不在编码器状态，不需要继续统计
    {
        s_smo_stable_count = 0U;
    }

    /*
     * 无感态对SMO非有限、失效、方向相反或严重角度分歧快速回退，
     *      高速虽然由SMO控制，但一旦SMO不可靠，就立即恢复有感控制
     */
    if ((state == motor_state_smo_run) &&
        ((Motor_SMO_IsHealthy() == 0U) ||
         (fabsf(SMO_Output.speed_rpm) < SMO_SWITCH_DOWN_RPM) ||
         (fabsf(g_motor_speed) < SMO_SWITCH_DOWN_RPM) ||
         (fabsf(g_encoder_smo_angle_error) > SMO_RUN_ANGLE_ERROR_MAX_RAD) ||
         (Motor_SpeedDirectionMatches(g_motor_speed, SMO_Output.speed_rpm) == 0U)))
    {
        Motor_SetState(motor_state_encoder_run);
        state = motor_state_encoder_run;
    }

    
    //根据本拍已经确定好的状态选择真正使用的角度
    switch(state)
    {
        case motor_state_align:
            /* 固定电角度进行转子对齐 */
            source_theta = 0.0f;
            break;
        case motor_state_open_loop:
            /* FOC开环运行，反转时电角度和Iq同时取反。 */
            if (foc_input.Dir == CW)
                open_loop_theta = open_loop_theta + Open_Loop_Step;
            else
                open_loop_theta = open_loop_theta - Open_Loop_Step;
            open_loop_theta = Motor_WrapAngle(open_loop_theta);/* 角度归一化到0~2pi */
            source_theta = open_loop_theta;
            break;
        case motor_state_encoder_run:
            /* 编码器有感FOC */
            source_theta = encoder_theta;
            break;
        case motor_state_smo_run:
            /* 高速无感FOC使用上一拍已收敛的PLL电角度。 */
            source_theta = SMO_Output.PLL_theta;
            break;
        default:
            return;
    }

    /* 用于判断是否发生有感无感的切换
    * s_fast_state：上一拍使用的状态
    * state ： 本拍使用的状态
    * s_angle_sync_offset：切换时角度偏移角
    */
    if (state != s_fast_state)
    {
        if ((Motor_StateUsesFoc(s_fast_state) != 0U) && (state != motor_state_align))
        {
            s_angle_sync_offset = Motor_AngleDifference(g_foc_elec_angle, source_theta);//切换时角度偏移角归一化
        }
        else
        {
            s_angle_sync_offset = 0.0f;
        }
        s_fast_state = state;
    }

    // 将本拍最终决定的角度交给FOC使用，FOC内部会使用本拍电流和上一拍电压更新SMO，刷新双角度误差。
    g_foc_elec_angle = Motor_WrapAngle(source_theta + s_angle_sync_offset);
    Foc_Run_Theta(g_foc_elec_angle);

    /* SMO已经用本拍数据更新，重新得到最新角度误差 */
    g_encoder_smo_angle_error = Motor_AngleDifference(encoder_theta, SMO_Output.PLL_theta);

    /*  切换时角度误差小直接归零，误差过大则平滑地减小到0  */
    if (fabsf(s_angle_sync_offset) <= ANGLE_SYNC_EPSILON_RAD)
    {
        s_angle_sync_offset = 0.0f;
    }
    else
    {
        float offset_step = s_angle_sync_offset * (1.0f - ANGLE_SYNC_DECAY);
        if (offset_step > ANGLE_SYNC_MAX_STEP_RAD)
            s_angle_sync_offset -= ANGLE_SYNC_MAX_STEP_RAD;
        else if (offset_step < -ANGLE_SYNC_MAX_STEP_RAD)
            s_angle_sync_offset += ANGLE_SYNC_MAX_STEP_RAD;
        else
            s_angle_sync_offset *= ANGLE_SYNC_DECAY;
    }

    /* SMO若在本拍观察过程中失效，立即提交回退供下一PWM周期使用。 */
    if ((g_motor_state == motor_state_smo_run) &&
        ((Motor_SMO_IsHealthy() == 0U) ||
         (fabsf(SMO_Output.speed_rpm) < SMO_SWITCH_DOWN_RPM) ||
         (fabsf(g_motor_speed) < SMO_SWITCH_DOWN_RPM) ||
         (fabsf(g_encoder_smo_angle_error) > SMO_RUN_ANGLE_ERROR_MAX_RAD) ||
         (Motor_SpeedDirectionMatches(g_motor_speed, SMO_Output.speed_rpm) == 0U)))
    {
        Motor_SetState(motor_state_encoder_run);
    }

}

