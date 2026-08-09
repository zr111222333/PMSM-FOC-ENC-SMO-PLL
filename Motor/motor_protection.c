#include "motor_protection.h"

volatile Motor_Protect_t g_protect;
static float s_phase_current_lpf[3];
static uint16_t s_phase_loss_cnt;

/*  硬件保护初始化  */
void Motor_Protect_Init(void)
{
    g_protect.Fault=Fault_NONE;
    g_protect.Over_Temp_Thr=80.0f;
    g_protect.Over_Current_Thr=4.0f;
    g_protect.Over_Voltage_Thr=30.0f;
    g_protect.Under_Voltage_Thr=18.0f;
    g_protect.Speed_Min_Rpm=1.0f;
    g_protect.Stall_cnt=0;   //堵转计数值
    g_protect.Stall_Thr=3000;   //3s: 堵转计数周期时间
    s_phase_current_lpf[0] = s_phase_current_lpf[1] = s_phase_current_lpf[2] = 0.0f;
    s_phase_loss_cnt = 0U;
}

/* 统一故障关断入口：故障锁存，必须重新初始化后才允许再次启动。 */
void Motor_Protect_Trip(Motor_Fault_e fault)
{
    /*  没有故障，或者已经存在故障时，不再重复处理  */
    if ((fault == Fault_NONE) || (g_protect.Fault != Fault_NONE))
    {
        return;
    }

    g_protect.Fault = fault;
    foc_input.Id_ref = 0.0f;
    foc_input.Iq_ref = 0.0f;

    PMSM_PWM_Duty_Set(0, 0, 0);
    PMSM_PWM_Stop();
    Motor_SetState(motor_state_fault);
}


/* 每次电流环执行一次（10kHz），仅做计算量很小的快速过流判断。 */
void Motor_Protect_FastCheck(void)
{
    Motor_State state = Motor_GetState();
    float i_abs;
    /* 已有故障，或者电机不在对齐、参数辨识、开环、闭环运行状态时，不检查  */
    if ((g_protect.Fault != Fault_NONE) || ((state != motor_state_align) && (state != motor_state_identify) && (state != motor_state_open_loop) 
                                        && (state != motor_state_encoder_run) && (state != motor_state_smo_run)))
    {
        return;
    }
    /*   检查软件要求驱动开启时，PF10是否被硬件拉成关断状态   */
    if (PMSM_Driver_ShutdownDetected() != 0U)
    {
        Motor_Protect_Trip(Fault_Driver_Shutdown);
        return;
    }

    
    i_abs = fmaxf(fabsf(I_abc.Ia), fmaxf(fabsf(I_abc.Ib), fabsf(I_abc.Ic)));
    if (i_abs > g_protect.Over_Current_Thr)
    {
        Motor_Protect_Trip(Fault_Over_Current);
    }
}


void Motor_Protect_Check(void)
{
    /* 堵转必须使用一直有效的编码器速度，不能使用低速可能清零的SMO速度 */
    float speed_rpm = g_motor_speed;
    // float speed_rpm = g_PMSM_Speed_NowUse_rpm;
    //目标速度与当前速度变化太大，则使用阶梯变化
    uint8_t speed_ref_is_ramping = 
        (fabsf(g_speed_ref_rpm - g_speed_ref_now_rpm) > SPEED_REF_RAMP_STEP_RPM) ? 1U : 0U;
    float i_abs = fmaxf(fabsf(I_abc.Ia),fmaxf(fabsf(I_abc.Ib), fabsf(I_abc.Ic))); 
    float temp = Motor_GetTemperature();
    float vbus = ADC_Raw.Vbus_raw / ADC_FULL_SCALE * ADC_REF_VOLTAGE * VBUS_DIVIDER_GAIN;
    Motor_Fault_e fault = Fault_NONE;

    Motor_State state = Motor_GetState();
    /* 首个故障锁存；非功率运行状态不执行软件保护。 */
    if (g_protect.Fault != Fault_NONE)
    {
        return;
    }
    if ((state != motor_state_align) &&
        (state != motor_state_identify) &&
        (state != motor_state_open_loop) &&
        (state != motor_state_encoder_run)&&
        (state != motor_state_smo_run))
    {
        g_protect.Stall_cnt = 0;
        s_phase_current_lpf[0] = s_phase_current_lpf[1] = s_phase_current_lpf[2] = 0.0f;
        s_phase_loss_cnt = 0U;
        return;
    }

    /* 只在闭环转速跟稳后统计，启动、低速和正反转过渡时清零。 */
    if (((state == motor_state_encoder_run) || (state == motor_state_smo_run)) &&//闭环转速跟稳后才统计，开环状态下三相电流本来就可能不平衡
        (speed_ref_is_ramping == 0U) &&     //速度不能正在斜坡变化，正反转经过0rpm本来就是正常过程，不能在这个阶段累计堵转时间
        (fabsf(speed_rpm) >= PHASE_LOSS_SPEED_MIN_RPM) &&//低速时电流波形不稳定
        (speed_rpm * g_speed_ref_now_rpm > 0.0f) &&     //实际转向必须和目标转向相同
        (fabsf(g_speed_ref_now_rpm - speed_rpm) <= PHASE_LOSS_SPEED_ERROR_MAX_RPM))//当前速度已经接近目标速度，转速尚未跟稳时禁止检测
    {
        float i_min;
        float i_mid;
        float i_max;                  
        //滤波值 = (1-α) * 滤波值 + α * 当前值              取绝对值因为三相电流是交流量，会不断正负变化。缺相检测关心的是电流幅值，不关心方向
        s_phase_current_lpf[0] = (1.0f - PHASE_LOSS_LPF_ALPHA) * s_phase_current_lpf[0] + PHASE_LOSS_LPF_ALPHA * fabsf(I_abc.Ia);
        s_phase_current_lpf[1] = (1.0f - PHASE_LOSS_LPF_ALPHA) * s_phase_current_lpf[1] + PHASE_LOSS_LPF_ALPHA * fabsf(I_abc.Ib);
        s_phase_current_lpf[2] = (1.0f - PHASE_LOSS_LPF_ALPHA) * s_phase_current_lpf[2] + PHASE_LOSS_LPF_ALPHA * fabsf(I_abc.Ic);
        //滤波以后比较的是一段时间内的平均绝对电流

        //三相电流绝对值排序，找出最小值、中间值和最大值
        i_min = fminf(s_phase_current_lpf[0], fminf(s_phase_current_lpf[1], s_phase_current_lpf[2]));
        i_max = fmaxf(s_phase_current_lpf[0], fmaxf(s_phase_current_lpf[1], s_phase_current_lpf[2]));
        i_mid = s_phase_current_lpf[0] + s_phase_current_lpf[1] + s_phase_current_lpf[2] - i_min - i_max;
        //缺相判断：另外两相必须确实存在一定电流,最小相必须明显小于正常相(最小相电流 < 中间相电流的30%)
        if ((i_mid >= PHASE_LOSS_CURRENT_MIN_A) && (i_min < i_mid * PHASE_LOSS_CURRENT_RATIO))
        {
            if (++s_phase_loss_cnt >= PHASE_LOSS_CONFIRM_COUNT)//连续约200ms才确认缺相
                fault = Fault_Phase_Loss;
        }
        else
            s_phase_loss_cnt = 0U;
    }
    else
    {//低速、正反转过渡、转速尚未跟稳时禁止检测，清零计数
        s_phase_current_lpf[0] = s_phase_current_lpf[1] = s_phase_current_lpf[2] = 0.0f;
        s_phase_loss_cnt = 0U;
    }


    if (PMSM_Driver_ShutdownDetected() != 0U)
        fault = Fault_Driver_Shutdown;
    else if(i_abs > g_protect.Over_Current_Thr)
        fault = Fault_Over_Current;
    else if(temp > g_protect.Over_Temp_Thr)
        fault = Fault_Over_Temp;
    else if(vbus > g_protect.Over_Voltage_Thr)
        fault = Fault_Over_Voltage;
    else if(vbus < g_protect.Under_Voltage_Thr)
        fault = Fault_Under_Voltage;
    //堵转
    else if (speed_ref_is_ramping != 0U)//正反转经过0rpm本来就是正常过程，不能在这个阶段累计堵转时间
    {
        /* 正在加减速或正反转过渡，不认为是堵转 */
        g_protect.Stall_cnt = 0U;
    }
    else if ((state == motor_state_open_loop || state == motor_state_encoder_run || state == motor_state_smo_run)
                                    && fabsf(speed_rpm) < g_protect.Speed_Min_Rpm && fabsf(foc_input.Iq_ref) > 0.1f)
    {
        g_protect.Stall_cnt++;
        if(g_protect.Stall_cnt > g_protect.Stall_Thr)
            fault = Fault_Stall;
    }
    else
        g_protect.Stall_cnt=0;

    if (fault != Fault_NONE)
    {
        Motor_Protect_Trip(fault);
    }
}




