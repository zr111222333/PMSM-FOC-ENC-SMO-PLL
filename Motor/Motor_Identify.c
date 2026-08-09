/*
 *@ brief: 检查电机是否停止
         → 检查母线电压
         → 测3组Rs
         → 发8次Ls电压脉冲
         → 计算平均值
         → 更新SMO参数和电流环PI
         → 成功或失败退出
 *
*/
#include "Motor_Identify.h"

#define IDENTIFY_TS_S                 0.0001f/* 参数辨识快速任务周期：100 us，对应10 kHz电流环 */
#define IDENTIFY_TOTAL_TIMEOUT_MS     3000U/* 整个参数辨识流程最大允许时间：3000 ms，超时则辨识失败 */
#define IDENTIFY_STANDSTILL_RPM       10.0f/* 启动辨识允许的最大机械转速：低于10 rpm才认为电机基本停止 */
#define IDENTIFY_CURRENT_LIMIT_A      2.0f/* 参数辨识期间允许的最大电流：超过2 A立即停止辨识 */
#define RS_POINT_COUNT                3U/* Rs辨识电流点数量：共使用3个不同的Id电流点 */
#define RS_STABLE_ERROR_A             0.08f/* 判断Id达到稳定的最大允许误差：实际Id与目标Id误差小于0.08 A */
#define RS_STABLE_TICKS               200U/* Id连续稳定时间：200个快速周期，即200 × 100 us = 20 ms */
#define RS_SAMPLE_TICKS               300U/* 每个Rs电流点的采样次数：300次，即采样时间为30 ms */
#define RS_POINT_TIMEOUT_TICKS        6000U/* 单个Rs电流点最大等待时间：6000个周期，即600 ms */
#define RS_MIN_OHM                    0.02f/* Rs辨识结果最小合理值：0.02 Ω */
#define RS_MAX_OHM                    5.0f/* Rs辨识结果最大合理值：5 Ω */
#define LS_PULSE_COUNT                8U/* Ls辨识电压脉冲次数：共进行8次并取平均值 */
#define LS_PULSE_VOLTAGE_V            1.0f/* Ls辨识施加的d轴电压脉冲幅值：1 V */
#define LS_PULSE_TICKS                3U/* 每次电压脉冲持续时间：3个快速周期，即0.3 ms */
#define LS_OFF_TICKS                  50U/* 两次电压脉冲之间的关断等待时间：50个周期，即5 ms */
#define LS_OFF_TIMEOUT_TICKS          200U/* 等待电流回落的最大时间：200个周期，即20 ms */
#define LS_ZERO_CURRENT_A             0.15f/* 允许开始下一次Ls脉冲的最大剩余电流：|Id|小于0.15 A */
#define LS_MIN_DELTA_CURRENT_A        0.05f/* 有效Ls计算要求的最小电流变化量：|ΔId|不能小于0.05 A */
#define LS_MIN_H                      0.00001f/* Ls辨识结果最小合理值：0.00001 H，即0.01 mH */
#define LS_MAX_H                      0.01000f/* Ls辨识结果最大合理值：0.01 H，即10 mH */

volatile Motor_Identify_Result g_motor_identify;

static const float s_rs_current_points[RS_POINT_COUNT] = {0.6f, 1.0f, 1.4f};// Rs辨识使用的3个不同Id电流点，单位A
static uint8_t s_attempted, s_reported, s_rs_index, s_ls_pulse, s_ls_phase;// 参数辨识是否已尝试、是否已报告、当前Rs电流点索引、当前Ls脉冲次数、当前Ls阶段
static uint16_t s_phase_ticks, s_stable_ticks, s_sample_ticks;// 当前阶段的快速周期计数、Id稳定计数、采样计数
static uint32_t s_start_ms; // 参数辨识开始时间，单位ms
static float s_sum_id, s_sum_vd, s_sum_rs, s_sum_ls, s_ls_i_start;// 当前阶段的Id累加、Vd累加、Rs累加、Ls累加、Ls脉冲开始时的Id值


//判断参数辨识值是否在合理区间，合理为0，不合理为1。
static uint8_t Identify_ValueInRange(float value, float min_value, float max_value) {
    return (!isnan(value) && !isinf(value) && (value >= min_value) && (value <= max_value)) ? 1U : 0U;
}

//参数辨识开始默认使用的参数
static void Identify_ApplyDefaults(void) 
{
    g_Motor_Rs = Motor_Rs_default;
    g_Motor_Ls = Motor_Ls_default;
    g_current_Kp = I_Kp_defult;
    g_current_Ki = I_Ki_default;
    PID_Init(&g_current_pid_Id, g_current_Kp, g_current_Ki, I_Kd, I_Kb, I_OutMax, I_OutMin);
    PID_Init(&g_current_pid_Iq, g_current_Kp, g_current_Ki, I_Kd, I_Kb, I_OutMax, I_OutMin);
}

//参数辨识初始化
void Motor_Identify_Init(void) 
{
    s_attempted = s_reported = 0U;// 参数辨识是否已尝试、是否已报告
    g_motor_identify.State = motor_identify_idle;
    g_motor_identify.Fail = motor_identify_fail_none;
    Identify_ApplyDefaults();// 使用默认参数
    g_motor_identify.Rs = g_Motor_Rs;// 更新结果结构体中的Rs和Ls为当前SMO参数
    g_motor_identify.Ls = g_Motor_Ls;
}

//参数辨识开始，设置状态为初始化，清除计数器和累加器
void Motor_Identify_Start(void) 
{
    if (s_attempted != 0U)
        return;
    s_attempted = 1U;
    s_rs_index = s_ls_pulse = s_ls_phase = 0U;// 当前Rs电流点索引、当前Ls脉冲次数、当前Ls阶段
    s_phase_ticks = s_stable_ticks = s_sample_ticks = 0U;// 当前阶段的快速周期计数、Id稳定计数、采样计数
    s_sum_id = s_sum_vd = s_sum_rs = s_sum_ls = 0.0f;// 当前阶段的Id累加、Vd累加、Rs累加、Ls累加
    s_start_ms = HAL_GetTick();// 参数辨识开始时间，单位ms
    g_motor_identify.Fail = motor_identify_fail_none;
    __DMB();// 数据内存屏障，确保前面的写入操作完成后再执行后续操作
    g_motor_identify.State = motor_identify_init;
}


//参数辨识停止PWM输出，清除Id、Iq参考值
static void Identify_Stop(void) 
{
    foc_input.Id_ref = 0.0f;
    foc_input.Iq_ref = 0.0f;
    PMSM_PWM_Duty_Set(0U, 0U, 0U);
    PMSM_PWM_Stop();
}

//参数辨识成功处理函数，更新SMO参数和电流环PI，并更新结果结构体
static void Identify_Succeed(float rs, float ls) 
{
    g_Motor_Rs = rs;
    g_Motor_Ls = ls;
    g_current_Kp = ls * CURRENT_LOOP_OMEGA_C;
    g_current_Ki = rs * CURRENT_LOOP_OMEGA_C * CURRENT_LOOP_TS_S;
    //把新的 Kp、Ki 分别写入 Id 环和 Iq 环，同时清除旧的积分和历史误差，避免原来 PI 的积分残留影响重新启动
    PID_Init(&g_current_pid_Id, g_current_Kp, g_current_Ki, I_Kd, I_Kb, I_OutMax, I_OutMin);
    PID_Init(&g_current_pid_Iq, g_current_Kp, g_current_Ki, I_Kd, I_Kb, I_OutMax, I_OutMin);
    g_motor_identify.Rs = rs;
    g_motor_identify.Ls = ls;
    g_motor_identify.Fail = motor_identify_fail_none;
    g_motor_identify.State = motor_identify_done;
    Identify_Stop();
}

//参数辨识失败处理函数，停止PWM输出，应用默认参数，并更新结果结构体
static void Identify_Fail(Motor_Identify_Fail reason) 
{
    Identify_ApplyDefaults();
    g_motor_identify.Rs = g_Motor_Rs;
    g_motor_identify.Ls = g_Motor_Ls;
    g_motor_identify.Fail = reason;
    g_motor_identify.State = motor_identify_failed;
    Identify_Stop();
}

//参数辨识检查电机是否停止、母线电压是否正常、相电流是否超过限制.正常为1，不正常为0
static uint8_t Identify_SafetyCheck(void) 
{
    float phase_current = fmaxf(fabsf(I_abc.Ia), fmaxf(fabsf(I_abc.Ib), fabsf(I_abc.Ic)));
    if (g_protect.Fault != Fault_NONE) {
        Identify_Fail(motor_identify_fail_protection);
        return 0U;
    }
    if (phase_current > IDENTIFY_CURRENT_LIMIT_A) {
        Identify_Fail(motor_identify_fail_current_limit);
        return 0U;
    }
    if (fabsf(g_motor_speed) > IDENTIFY_STANDSTILL_RPM) {
        Identify_Fail(motor_identify_fail_not_stopped);
        return 0U;
    }
    return 1U;
}

/* 
 * Rs辨识阶段的快速任务函数，逐个采样Rs电流点，计算平均值，并检查是否超时或超出合理范围
 *  先判断Id、Iq差值是否稳定，稳定后再计算出RS。
 *
*/ 
static void Identify_RsTask(void) 
{
    float target = s_rs_current_points[s_rs_index];// 当前Rs电流点目标值
    float rs_point;// 当前Rs电流点计算值
    foc_input.Id_ref = target;
    foc_input.Iq_ref = 0.0f;
    Foc_Run_Theta(0.0f);// 使用固定电角度0，转子对齐阶段使用
    if (Identify_SafetyCheck() == 0U)// 安全检查失败，立即退出
        return;
    s_phase_ticks++;// 当前阶段快速周期计数+1
    if ((fabsf(I_dq.Id - target) <= RS_STABLE_ERROR_A) && (fabsf(I_dq.Iq) <= RS_STABLE_ERROR_A)) 
    {// Id和Iq都稳定在目标值附近
        if (s_stable_ticks < RS_STABLE_TICKS)
            s_stable_ticks++;
        else // Id已经连续稳定在目标值附近超过RS_STABLE_TICKS个快速周期，开始采样
        {
            s_sum_id += I_dq.Id;
            s_sum_vd += V_dq.Vd;
            s_sample_ticks++;
        }
    } 
    else // Id或Iq不稳定，重置稳定计数和采样计数
    {
        s_stable_ticks = 0U;
        s_sample_ticks = 0U;
        s_sum_id = 0.0f;
        s_sum_vd = 0.0f;
    }
    // 如果Id稳定在目标值附近，开始采样，采样次数达到RS_SAMPLE_TICKS后计算当前Rs电流点的值
    if (s_sample_ticks >= RS_SAMPLE_TICKS) 
    {
        rs_point = fabsf(s_sum_vd / s_sum_id);// 计算当前Rs电流点的值
        // 检查当前Rs电流点的值是否在合理范围内，如果不在范围内，立即失败退出
        if (Identify_ValueInRange(rs_point, RS_MIN_OHM, RS_MAX_OHM) == 0U) 
        {
            Identify_Fail(motor_identify_fail_rs_range);
            return;
        }
        s_sum_rs += rs_point;// 累加当前Rs电流点的值，用于后续计算平均值
        s_rs_index++;// 当前Rs电流点索引+1，准备采样下一个电流点
        s_phase_ticks = s_stable_ticks = s_sample_ticks = 0U;// 重置计数器
        s_sum_id = s_sum_vd = 0.0f;// 重置累加值
        // 如果已经采样了RS_POINT_COUNT个电流点，计算平均值并进入Ls辨识阶段，否则继续采样下一个电流点
        if (s_rs_index >= RS_POINT_COUNT) 
        {
            g_motor_identify.Rs = s_sum_rs / (float)RS_POINT_COUNT;
            g_motor_identify.State = motor_identify_ls;
            s_ls_phase = 0U;// 进入Ls辨识阶段，初始化Ls阶段计数器
        }
    } 
    else if (s_phase_ticks >= RS_POINT_TIMEOUT_TICKS)// 如果当前Rs电流点等待时间超过RS_POINT_TIMEOUT_TICKS个快速周期，立即失败退出
        Identify_Fail(motor_identify_fail_rs_timeout);
}


/* 
 * Ls辨识阶段的快速任务函数，通过脉冲电压激励和电流测量来计算电感值
 *    根据LS公式可以知道，Vd需要在正负来回变化，同时引出电流变化所以对电流求导
 *
*/ 
static void Identify_LsTask(void) 
{
    float sign = ((s_ls_pulse & 1U) == 0U) ? 1.0f : -1.0f;// 奇偶脉冲交替施加正负电压
    float pulse_voltage = sign * LS_PULSE_VOLTAGE_V;      // 当前脉冲电压值
    float command_vd = ((s_ls_phase != 0U) && (s_phase_ticks < LS_PULSE_TICKS)) ? pulse_voltage : 0.0f;// 当前命令电压值，只有在Ls脉冲阶段且脉冲时间未结束时才施加脉冲电压，否则为0
    Foc_Run_Voltage(command_vd, 0.0f, 0.0f);
    if (Identify_SafetyCheck() == 0U)
        return;
    if (s_ls_phase == 0U) // Ls脉冲阶段，等待电流回落到接近0的状态
    {
        s_phase_ticks++;
        if ((s_phase_ticks >= LS_OFF_TICKS) && (fabsf(I_dq.Id) <= LS_ZERO_CURRENT_A)) // 如果等待时间超过LS_OFF_TICKS且Id接近0，进入Ls脉冲阶段
        {
            s_ls_phase = 1U;// 进入Ls脉冲阶段,正式施加 LS 电压脉冲的阶段
            s_phase_ticks = 0U;
        } 
        else if (s_phase_ticks >= LS_OFF_TIMEOUT_TICKS)// 如果等待时间太长，说明电流回落过慢，立即失败退出
            Identify_Fail(motor_identify_fail_ls_response);
        return;
    }
    if (s_phase_ticks == 0U)// 记录Ls脉冲开始时的Id值，用于后续计算Ls
        s_ls_i_start = I_dq.Id;
    if (s_phase_ticks < LS_PULSE_TICKS) // 如果Ls脉冲时间未结束，继续施加脉冲电压
    {
        s_phase_ticks++;
        return;//退出整个循环
    }
    else// 如果Ls脉冲时间结束，计算Ls值并累加，用于后续计算平均值
    {
        float delta_i = I_dq.Id - s_ls_i_start;
        float i_average = 0.5f * (I_dq.Id + s_ls_i_start);// 计算Ls脉冲期间的平均电流
        float ls;
        if (fabsf(delta_i) < LS_MIN_DELTA_CURRENT_A) // 如果电流变化量太小，说明Ls脉冲没有有效激励电流，立即失败退出
        {
            Identify_Fail(motor_identify_fail_ls_response);
            return;
        }
        ls = (pulse_voltage - g_motor_identify.Rs * i_average) * (LS_PULSE_TICKS * IDENTIFY_TS_S) / delta_i;// 根据LS公式计算Ls值
        if (isnan(ls) || isinf(ls))// 如果计算结果为NaN或Inf，说明计算异常，立即失败退出
        {
            Identify_Fail(motor_identify_fail_ls_response);
            return;
        }
        s_sum_ls += ls;// 累加Ls值，用于后续计算平均值
    }
    s_ls_pulse++;// Ls脉冲次数+1
    s_ls_phase = 0U;// 重置Ls阶段为等待电流回落阶段
    s_phase_ticks = 0U;// 重置Ls脉冲阶段计数器
    if (s_ls_pulse >= LS_PULSE_COUNT) // 如果Ls脉冲次数达到LS_PULSE_COUNT，计算平均值并检查是否在合理范围内
    {
        float ls_average = s_sum_ls / (float)LS_PULSE_COUNT;// 计算Ls平均值
        if (Identify_ValueInRange(ls_average, LS_MIN_H, LS_MAX_H) == 0U)
            Identify_Fail(motor_identify_fail_ls_range);
        else
            Identify_Succeed(g_motor_identify.Rs, ls_average);
    }
}

// 参数辨识快速任务函数，根据当前状态调用对应的任务函数，并检查总超时
void Motor_Identify_FastTask(void) 
{
    if (g_motor_identify.State == motor_identify_idle) 
    {
        Foc_Run_Voltage(0.0f, 0.0f, 0.0f);
        return;
    }
    if ((HAL_GetTick() - s_start_ms) >= IDENTIFY_TOTAL_TIMEOUT_MS) // 如果总时间超过IDENTIFY_TOTAL_TIMEOUT_MS，立即失败退出
    {
        Identify_Fail(motor_identify_fail_total_timeout);
        return;
    }
    if (g_motor_identify.State == motor_identify_init)
    {
        Foc_Run_Voltage(0.0f, 0.0f, 0.0f);
        if (Identify_SafetyCheck() == 0U)
            return;
        if ((foc_input.Udc < g_protect.Under_Voltage_Thr) || (foc_input.Udc > g_protect.Over_Voltage_Thr))// 如果母线电压不在18~30V范围内，立即失败退出
            Identify_Fail(motor_identify_fail_vbus);
        else
            g_motor_identify.State = motor_identify_rs;
    } 
    else if (g_motor_identify.State == motor_identify_rs)// 如果当前状态为Rs辨识阶段，调用Identify_RsTask()函数
        Identify_RsTask();
    else if (g_motor_identify.State == motor_identify_ls)
        Identify_LsTask();
}

// 参数辨识中止函数，如果当前状态为初始化、Rs辨识或Ls辨识阶段，调用Identify_Fail()函数并传入中止原因
void Motor_Identify_Abort(Motor_Identify_Fail reason) 
{
    if ((g_motor_identify.State == motor_identify_init) ||
        (g_motor_identify.State == motor_identify_rs) ||
        (g_motor_identify.State == motor_identify_ls))
        Identify_Fail(reason);// 调用Identify_Fail()函数并传入中止原因
}

// 参数辨识结果报告函数，如果当前状态为完成或失败阶段且未报告过，打印结果并设置已报告标志
uint8_t Motor_Identify_WasAttempted(void) 
{ 
    return s_attempted; // 返回参数辨识是否已尝试过
}

// 参数辨识结果报告函数，看参数辨识是成功还是失败
uint8_t Motor_Identify_IsFinished(void) 
{
    return ((g_motor_identify.State == motor_identify_done) || (g_motor_identify.State == motor_identify_failed)) ? 1U : 0U;
}

// 参数辨识结果报告函数，如果当前状态为完成或失败阶段且未报告过，打印结果并设置已报告标志
void Motor_Identify_ReportOnce(void) 
{
    static const char *const reason_text[] = 
    {
        "NONE", "NOT_STOPPED", "VBUS", "CURRENT_LIMIT", "RS_TIMEOUT",
        "RS_RANGE", "LS_RESPONSE", "LS_RANGE", "TOTAL_TIMEOUT", "PROTECTION"
    };
    if ((Motor_Identify_IsFinished() == 0U) || (s_reported != 0U))// 如果参数辨识未完成或已报告过，直接返回
        return;
    s_reported = 1U;// 设置已报告标志，避免重复打印
    printf("[IDENTIFY] state=%s, Rs=%.6f ohm, Ls=%.9f H, reason=%s\r\n",
           (g_motor_identify.State == motor_identify_done) ? "DONE" : "FAILED",
           g_motor_identify.Rs, g_motor_identify.Ls,
           reason_text[(uint8_t)g_motor_identify.Fail]);
}

