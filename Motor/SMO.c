#include "SMO.h"

volatile float g_Motor_Rs = Motor_Rs_default;//
volatile float g_Motor_Ls = Motor_Ls_default;

SMO_Input_Def SMO_Input = {0};
volatile SMO_Output_Def SMO_Output = {0};

/* 下一时刻预测电流 i_hat[k+1] */
static float i_pred[2] = {0.0f, 0.0f};
static float Wc = 0.0f;  //截止频率
static float PLL_Integral = 0.0f;/* PLL积分项 */
/* 当前实际旋转方向：1正转，-1反转；过零时保持上一次方向 */
static float s_dir_sign = 1.0f;
/* SMO_Init后的首拍仅建立“上一PWM周期电压”时序，不运行观测器 */
static uint8_t s_voltage_history_ready = 0U;

/*
 * @brief sat边界层函数
 *
 * 电流误差大于边界层时输出±1；
 * 电流误差位于边界层内时线性变化。
 */
static float sat(float error)
{
    if (error > SMO_Boundary)
    {
        return 1.0f;
    }
    else if (error < -SMO_Boundary)
    {
        return -1.0f;
    }
    else
        return error / SMO_Boundary;    //输出要求始终归一化
}

void SMO_Init(void)
{

    Wc = -logf(SMO_LPF_Alpha) / SMO_Ts;  //截止频率
    PLL_Integral = 0.0f;
    i_pred[0] = i_pred[1] = 0.0f;
    SMO_Output.PLL_speed_delta = 0.0f;
    s_dir_sign = 1.0f;
    s_voltage_history_ready = 0U;//SMO刚初始化还没有建立可靠的上一周期电压

    SMO_Input.I_alpha = 0.0f;
    SMO_Input.I_beta = 0.0f;
    SMO_Input.V_alpha = 0.0f;
    SMO_Input.V_beta = 0.0f;

    SMO_Output.theta = 0.0f;
    SMO_Output.theta_last = 0.0f;
    SMO_Output.speed = 0.0f;
    SMO_Output.speed_rpm = 0.0f;

    SMO_Output.E_alpha = 0.0f;
    SMO_Output.E_beta  = 0.0f;
    SMO_Output.E_alpha_lpf = 0.0f;
    SMO_Output.E_beta_lpf  = 0.0f;
    SMO_Output.E_alpha_comp = 0.0f;
    SMO_Output.E_beta_comp = 0.0f;
    SMO_Output.first_run = 1;

    SMO_Output.Ed = 0.0f;
    SMO_Output.Ed_error = 0.0f;
    SMO_Output.PLL_theta = 0.0f;
    SMO_Output.PLL_speed = 0.0f;

    SMO_Output.I_alpha_hat = 0.0f;
    SMO_Output.I_beta_hat = 0.0f;
    SMO_Output.I_alpha_err = 0.0f;
    SMO_Output.I_beta_err = 0.0f;
    SMO_Output.input_valid = 0U;//初始化完成不等于SMO输入已经有效。必须实际获得一拍正常电流和电压后，才能置1
    SMO_Output.angle_valid = 0U;//初始化时SMO角度不能使用
}


/*
 * @brief 执行一次SMO和PLL计算
 *
 * 执行顺序：
 * 1. 计算估算电流误差
 * 2. 计算滑模切换校正量
 * 3. 对切换量低通滤波，得到反电动势
 * 4. 预测下一拍电流
 * 5. 根据反电动势计算角度和速度
 * 6. 使用PLL进一步估算角度和速度
 */
void SMO_Run(const SMO_Input_Def *input, volatile SMO_Output_Def *output)
{

    /* 0. 输入和内部状态NaN/Inf(非数值/无穷大)常判断 */
    if (isnan(input->I_alpha) || isinf(input->I_alpha) ||
        isnan(input->I_beta)  || isinf(input->I_beta)  ||
        isnan(input->V_alpha) || isinf(input->V_alpha) ||
        isnan(input->V_beta)  || isinf(input->V_beta)  ||
        isnan(output->I_alpha_hat) || isinf(output->I_alpha_hat) ||
        isnan(output->I_beta_hat)  || isinf(output->I_beta_hat)  ||
        isnan(output->E_alpha_lpf) || isinf(output->E_alpha_lpf) ||
        isnan(output->E_beta_lpf)  || isinf(output->E_beta_lpf)  ||
        isnan(output->PLL_theta)   || isinf(output->PLL_theta)   ||
        isnan(output->PLL_speed)   || isinf(output->PLL_speed))
    {
        // 输入异常，重置观测器状态防止毒瘤蔓延
        SMO_Init(); 
        output->input_valid = 0;
        output->angle_valid = 0;
        return; // 直接跳过本拍计算
    }
    output->input_valid = 1;
    uint8_t pll_sync_now = 0U;  //synchronize同步：PLL是否在当前这一拍刚完成同步

    /* 1. 计算当前时刻电流估算误差
     *
     * s[k] = i_hat[k] - i_measured[k]；估计-观测
     */
    output->I_alpha_err = output->I_alpha_hat - input->I_alpha;
    output->I_beta_err  = output->I_beta_hat - input->I_beta;

    /* 2. 计算反电动势校正量
     * e_switch = Ks × sat(s)
     */
    output->E_alpha = SMO_Ks * sat(output->I_alpha_err);
    output->E_beta  = SMO_Ks * sat(output->I_beta_err );

    /* 3. 对滑模切换量进行一阶低通滤波 
     *
     * 滤波后的平均值近似为反电动势
     */
    output->E_alpha_lpf = output->E_alpha * (1.0f - SMO_LPF_Alpha) + output->E_alpha_lpf * SMO_LPF_Alpha;
    output->E_beta_lpf  = output->E_beta  * (1.0f - SMO_LPF_Alpha) + output->E_beta_lpf  * SMO_LPF_Alpha;

     /* 4. 预测下一时刻的观测电流
     *
     * i_hat[k+1]= i_hat[k] + Ts/L × (v - R×i_hat[k] - e_switch)
     * 注意：
     * 电流预测使用原始切换量e_switch，
     * 不是使用低通后的e_lpf。
     */
    i_pred[0]=output->I_alpha_hat + SMO_Ts / Motor_Ls * (input->V_alpha - Motor_Rs * output->I_alpha_hat - output->E_alpha);
    i_pred[1]=output->I_beta_hat  + SMO_Ts / Motor_Ls * (input->V_beta  - Motor_Rs * output->I_beta_hat  - output->E_beta );

    /* 5. 根据反电动势估算电角速度--直接计算法
     * |E| = sqrt(Ealpha? + Ebeta?)
     * electrical_speed = |E| / flux
     *  注意：
     *  计算角度和PLL必须区分方向，否则反转时计算出的角度可能整体相差 π
     */
    /* 使用编码器实际速度判断电机旋转方向 
       反电动势幅值计算电角速度绝对值 */
    float speed_abs = sqrtf(output->E_alpha_lpf * output->E_alpha_lpf + output->E_beta_lpf  * output->E_beta_lpf) / Motor_Flux;

    /* 有感阶段用编码器判定方向；进入无感后锁存方向，避免依赖编码器反馈。 */
    if ((Motor_GetState() != motor_state_smo_run) && (g_motor_speed > 30.0f))
    {
        s_dir_sign = 1.0f;
    }
    else if ((Motor_GetState() != motor_state_smo_run) && (g_motor_speed < -30.0f))
    {
        s_dir_sign = -1.0f;
    }

    /* 得到带正负方向的电角速度 */
    output->speed = s_dir_sign * speed_abs;

    /* 反转时将反电动势矢量翻转回来 */
    float E_alpha_lpf_dir = s_dir_sign * output->E_alpha_lpf;
    float E_beta_lpf_dir  = s_dir_sign * output->E_beta_lpf;

    /* 根据带方向的反电动势计算转子电角度 */
    output->theta = -atan2f(E_alpha_lpf_dir, E_beta_lpf_dir) + atan2f(output->speed, Wc);

    /* 角度归一化到0～2π */
    while (output->theta >= 2.0f * PI)
    {
        output->theta -= 2.0f * PI;
    }
    while (output->theta < 0.0f)
    {
        output->theta += 2.0f * PI;
    }
    /* 低速反电动势不足、数据无效，SMO角度无效 */
    output->angle_valid = ((output->input_valid != 0U) && !isnan(speed_abs) && !isinf(speed_abs) &&
                           (speed_abs >= SMO_Min_valid_speed_elec)) ? 1U : 0U;

    /* 第一次有效运行时同步PLL */
    if ((output->first_run != 0U) && (output->angle_valid != 0U))
    {
        output->first_run = 0U;
        output->theta_last = output->theta;
        output->PLL_theta = output->theta;
        output->PLL_speed = output->speed;
        pll_sync_now = 1U;//第一次有效同步PLL
    }
    else
    {
        output->theta_last = output->theta;
    }
    
    //PLL
    
    /*  6. PLL使用的反电动势进行相位补偿  
     *     把低通后的反电动势矢量提前 phi
     */  
    float phi = atan2f(output->speed, Wc);  //低通相位补偿角
    output->E_alpha_comp = E_alpha_lpf_dir * cosf(phi) - E_beta_lpf_dir * sinf(phi);  //相位补偿后的alpha轴反电动势
    output->E_beta_comp  = E_alpha_lpf_dir * sinf(phi) + E_beta_lpf_dir * cosf(phi);  //相位补偿后的beta轴反电动势
    
    // PLL 用补偿后的反电动势算 ed
    output->Ed = output->E_alpha_comp * cosf(output->PLL_theta) + output->E_beta_comp * sinf(output->PLL_theta);  //补偿反电动势在PLL d轴上的原始误差

    /* 7. PLL相位误差
     *
     * 当PLL角度准确时，反电动势d轴分量Ed接近0
     */
    // output->Ed_error = 0.0f - output->Ed;
    // 计算补偿后反电势的幅值
    float E_mag = sqrtf(output->E_alpha_comp * output->E_alpha_comp + output->E_beta_comp * output->E_beta_comp);
    // 低速反电动势失效判断
    if ( output->angle_valid == 0) 
    {
        output->Ed_error = 0.0f; // 归一化时防除0，误差置0
    }
    else
    {
        output->Ed_error = -output->Ed / E_mag; // 新增：归一化 Ed，使其在[-1, 1]之间，与转速无关
    }

    /*  8. PLL PI控制器
     * PI输出作为速度修正量 
     */
    if (output->angle_valid == 0)
    {
        //低速失效时，彻底冻结PLL，清空积分历史，防止角度继续跑飞
        PLL_Integral = 0.0f;
        output->PLL_speed_delta = 0.0f;
        output->PLL_speed = 0.0f; 
        output->speed_rpm = 0.0f;
        output->first_run = 1U;
        // 注意：这里不更新 PLL_theta，让它冻结在失效前的最后一个可靠角度
    }
    else
    {
        PLL_Integral =PLL_Integral + PLL_Ki * output->Ed_error * SMO_Ts;
        if (PLL_Integral > PLL_Integral_Max) PLL_Integral = PLL_Integral_Max;
        if (PLL_Integral < PLL_Integral_Min) PLL_Integral = PLL_Integral_Min;
        output->PLL_speed_delta  = PLL_Kp * output->Ed_error + PLL_Integral;
        // PLL估算速度
        output->PLL_speed = output->speed + output->PLL_speed_delta;  //PLL估算电角速度 = 基础速度 + PLL修正速度
        // PLL速度限幅，防止追角时跑飞
        if (output->PLL_speed > SMO_Max_speed_elec) output->PLL_speed = SMO_Max_speed_elec;
        if (output->PLL_speed < -SMO_Max_speed_elec) output->PLL_speed = -SMO_Max_speed_elec;
        //PLL估算角度
        output->PLL_theta = output->PLL_theta + output->PLL_speed * SMO_Ts;
        //角度归一化
        if (output->PLL_theta < 0.0f)
        {
            output->PLL_theta += 2.0f * PI;
        }
        else if (output->PLL_theta > 2.0f * PI)
        {
            output->PLL_theta -= 2.0f * PI;
        }
        /* 电角速度转换为机械转速rpm，仅滤波反馈值，不影响PLL角度积分。 */
        float speed_rpm_raw = output->PLL_speed * 60.0f / (2.0f * PI * Motor_PolePairs);
        if (pll_sync_now != 0U)
        {   //第一次同步时直接输出真实估算速度,否则初始速度为0时，滤波会将速度降得太低。
            output->speed_rpm = speed_rpm_raw;
        }
        else//后续普通运行时再低通滤波
        {
            output->speed_rpm = speed_rpm_raw * (1.0f - SMO_SPEED_LPF_Alpha) + output->speed_rpm * SMO_SPEED_LPF_Alpha;
        }
    }

    /*  9. 更新观测电流
     *
     * 本次预测的k+1时刻电流，
     * 作为下一次调用时的k时刻观测电流。
     */
    output->I_alpha_hat = i_pred[0];
    output->I_beta_hat  = i_pred[1];

}


/*
 * @brief 从当前FOC变量获取SMO输入
 *
 * 必须在Clarke变换完成后、反Park变换之前调用：
 * 此时I_AlphaBeta为当前采样，V_AlphaBeta仍是上一PWM周期命令。
 */
 void SMO_Observe_Run(void)
{
    SMO_Input.I_alpha = I_AlphaBeta.I_alpha;
    SMO_Input.I_beta = I_AlphaBeta.I_beta;

    SMO_Input.V_alpha = V_AlphaBeta.V_alpha;
    SMO_Input.V_beta = V_AlphaBeta.V_beta;

    /* 初始化后的首拍丢弃旧电压，同时把观测电流同步到实测电流。 */
    if (s_voltage_history_ready == 0U)//SMO是否第一次运行,观测值没有可靠的上一PWM周期电压
    {
        if (isnan(SMO_Input.I_alpha) || isinf(SMO_Input.I_alpha) || isnan(SMO_Input.I_beta)  || isinf(SMO_Input.I_beta))
        {
            SMO_Init();
            return;
        }
        SMO_Output.I_alpha_hat = SMO_Input.I_alpha;//观测电流初值直接等于实际电流
        SMO_Output.I_beta_hat = SMO_Input.I_beta;
        SMO_Output.input_valid = 1U;
        SMO_Output.angle_valid = 0U;//虽然输入正常，但这一拍没有真正运行SMO，所以角度仍然不能使用
        s_voltage_history_ready = 1U;
        return;
    }   

    SMO_Run(&SMO_Input, &SMO_Output);
}

