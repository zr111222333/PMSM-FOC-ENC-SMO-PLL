#include "FOC.h"

FOC_Input         foc_input;
Voltage_DQ        V_dq;
Voltage_AlphaBeta V_AlphaBeta;
Current_ABC       I_abc;
Current_AlphaBeta I_AlphaBeta;
Current_DQ        I_dq;
SinCos_Cault      SinCos_Transf;

void Foc_Init(void)
{
    foc_input.Dir=CW;
    foc_input.Id_ref=0.0f;
    foc_input.Iq_ref=0.0f;
    foc_input.Tpwm=8400 * 2; // 中心对齐周期;
    foc_input.Udc=24.0f;    //初始给母线电压用于采集到真实母线给FOC使用
}

void Foc_Run_Core(float theta)
{
    /*     3相电流     */
    I_abc.Ia=(ADC_Raw.Iu_raw * 3.3f / 4096.0f ) / 0.12f;  //ADC_Raw.Iu_raw * 3.3f / 4096.0f - 1.25f
    I_abc.Ib=(ADC_Raw.Iv_raw * 3.3f / 4096.0f ) / 0.12f;  //1.25V是理论停机电压值，这里我们使用实际采集到的电压值，防止因为零偏不准导致的电流计算不准
    I_abc.Ic=(ADC_Raw.Iw_raw * 3.3f / 4096.0f ) / 0.12f;

    /* 先做10kHz快速保护，避免SMO运算延迟过流和驱动关断响应。 */
    Motor_Protect_FastCheck();
    if ((g_protect.Fault != Fault_NONE) ||
        (Motor_GetState() == motor_state_fault))
    {
        return;
    }

    /* 本次FOC使用的电角度 */
    foc_input.theta = theta;
    Theta_Transf_CosSin(foc_input.theta,&SinCos_Transf);

    /*     1. CLAEK变换     */
    Clark_Transf(&I_abc,&I_AlphaBeta);

    /*
     * 当前I_AlphaBeta配合尚未被本拍反Park覆盖的V_AlphaBeta，
     * 即Iαβ[k] + Vαβ[k-1]。所有有功FOC状态均在后台连续运行SMO。
     */
    SMO_Observe_Run();


    /*     2. Park变换     */
    Park_Transf(&I_AlphaBeta,foc_input.theta,&I_dq);

    /*     3. PI控制      */
    //线电压是相电压的√3倍，所以相电压就是线电压的1/√3倍,这样线电压是动态变化的，如果不这样动态变化的话，输出做好边界导致限制速度
    /* SVPWM线性区最大电压约为Udc / sqrt(3)，这里取0.55倍母线电压 */
    float voltage_limit = foc_input.Udc * 0.55f;
    /* 先计算Id环 */
    g_current_pid_Id.OutMax = voltage_limit;
    g_current_pid_Id.OutMin = -voltage_limit;
    // Id 环 (励磁环)
    g_current_pid_Id.SetPoint=foc_input.Id_ref;
    PID_Ctrl(&g_current_pid_Id,I_dq.Id);
    V_dq.Vd=g_current_pid_Id.Output;
    /* 根据Vd剩余电压计算Vq允许的最大值 */
    float vq_limit_square = voltage_limit * voltage_limit - V_dq.Vd * V_dq.Vd;
    float vq_limit = (vq_limit_square > 0.0f) ? sqrtf(vq_limit_square) : 0.0f;
    g_current_pid_Iq.OutMax = vq_limit;
    g_current_pid_Iq.OutMin = -vq_limit;
    // Iq 环 (转矩环)
    g_current_pid_Iq.SetPoint=foc_input.Iq_ref;
    PID_Ctrl(&g_current_pid_Iq,I_dq.Iq);
    V_dq.Vq=g_current_pid_Iq.Output;
   

    /*     4. RevPark变换     */
    RevPark_Transf(&V_dq,foc_input.theta,&V_AlphaBeta);
    
    /*     5. SVPWM     */
    SVPWM_Transf(&V_AlphaBeta,foc_input.Udc,foc_input.Tpwm);
}


/*
 * @brief 编码器有感FOC
*/
void Foc_Run(void)
{
    float encoder_theta =Encoder_ElecAngle(Encoder_elec_angel_read);
    Foc_Run_Core(encoder_theta);
}


/*
 * @brief 固定电角度FOC，转子对齐阶段使用
*/
void Foc_Run_Theta(float theta)
{
    Foc_Run_Core(theta);//开环角度自增
}

//参数定时使用，直接指定Vd、Vq,不经过PI获取Vd、Vq参数更真实
void Foc_Run_Voltage(float vd, float vq, float theta)
{
    I_abc.Ia = (ADC_Raw.Iu_raw * 3.3f / 4096.0f) / 0.12f;
    I_abc.Ib = (ADC_Raw.Iv_raw * 3.3f / 4096.0f) / 0.12f;
    I_abc.Ic = (ADC_Raw.Iw_raw * 3.3f / 4096.0f) / 0.12f;

    Motor_Protect_FastCheck();
    if ((g_protect.Fault != Fault_NONE) || (Motor_GetState() == motor_state_fault))
        return;
    foc_input.theta = theta;
    Theta_Transf_CosSin(theta, &SinCos_Transf);
    Clark_Transf(&I_abc, &I_AlphaBeta);
    Park_Transf(&I_AlphaBeta, theta, &I_dq);
    V_dq.Vd = vd;// 直接施加的d轴电压
    V_dq.Vq = vq;// 直接施加的q轴电压
    RevPark_Transf(&V_dq, theta, &V_AlphaBeta);
    SVPWM_Transf(&V_AlphaBeta, foc_input.Udc, foc_input.Tpwm);
}



/*    角度计算模块封装减少计算次数时间   */
void Theta_Transf_CosSin(float theta, SinCos_Cault* SinCos_Transf)
{
    SinCos_Transf->sin_theta=sinf(theta);
    SinCos_Transf->cos_theta=cosf(theta);
}

/*     RevPark变换     */
void RevPark_Transf(Voltage_DQ *V_dq_name,float theta,Voltage_AlphaBeta *V_AlphaBeta_name)
{
    V_AlphaBeta_name->V_alpha=V_dq_name->Vd * SinCos_Transf.cos_theta - V_dq_name->Vq * SinCos_Transf.sin_theta;
    V_AlphaBeta_name->V_beta= V_dq_name->Vd * SinCos_Transf.sin_theta + V_dq_name->Vq * SinCos_Transf.cos_theta;
}


/*     CLAEK变换     */
void Clark_Transf(Current_ABC *I_abc_name,Current_AlphaBeta *I_AlphaBeta_name)
{
    I_AlphaBeta_name->I_alpha = I_abc_name->Ia;
    I_AlphaBeta_name->I_beta  = (I_abc_name->Ia + 2.0f * I_abc_name->Ib) * 0.57735026919f; // 1/sqrt(3)
}

/*     Park变换     */
void Park_Transf(Current_AlphaBeta *I_AlphaBeta_name,float theta,Current_DQ *I_dq_name)
{
    I_dq_name->Id = I_AlphaBeta_name->I_alpha * SinCos_Transf.cos_theta + I_AlphaBeta_name->I_beta * SinCos_Transf.sin_theta;
    I_dq_name->Iq = -I_AlphaBeta_name->I_alpha *  SinCos_Transf.sin_theta + I_AlphaBeta_name->I_beta * SinCos_Transf.cos_theta;
}

/*     SVPWM     */
void SVPWM_Transf(Voltage_AlphaBeta *V_AlphaBeta_name,float Udc,float Tpwm)
{
    //欠压保护,防止母线电压过低导致计算的死区时间过长，电机无法正常工作
    if (Udc < 1.0f)
    {
        TIM1->CCR1 = 0;
        TIM1->CCR2 = 0;
        TIM1->CCR3 = 0;
        return;
    }
    /*    参考电压矢量的扇区判断    */
    float V_ref1,V_ref2,V_ref3,X,Y,Z,Tx,Ty,T0,Ta, Tb, Tc;
    int8_t A,B,C,N=0;
    V_ref1 = V_AlphaBeta_name->V_beta;
    V_ref2 = (1.7320508f * V_AlphaBeta_name->V_alpha -V_AlphaBeta_name->V_beta)/2; // sqrt(3)=1.7320508f
    V_ref3 = (-1.7320508f * V_AlphaBeta_name->V_alpha -V_AlphaBeta_name->V_beta)/2;
    if(V_ref1>0) A=1;
    else A=0;
    if(V_ref2>0) B=1;
    else B=0;
    if(V_ref3>0) C=1;
    else C=0;
    N=4*C+2*B+A;
    switch(N)
    {
        case 1://扇区2
            break;
        case 2://扇区6
            break;
        case 3://扇区1
            break;
        case 4://扇区4
            break;
        case 5://扇区3
            break;
        case 6://扇区5
            break;
    }
    /*    非零与零矢量作用时间   */
    X=(1.7320508f * Tpwm * V_AlphaBeta_name->V_beta) / Udc;
    Y=(0.8660254f * Tpwm * V_AlphaBeta_name->V_beta + 1.5f * Tpwm * V_AlphaBeta_name->V_alpha) / Udc;  // sqrt(3)/2=0.8660254f
    Z=(Tpwm / Udc) * (0.8660254f *V_AlphaBeta_name->V_beta - 1.5f * V_AlphaBeta_name->V_alpha);
    switch(N)
    {
        case 1://扇区2
            Tx=Z; Ty=Y; T0=(Tpwm-Tx-Ty)/2;
            break;
        case 2://扇区6
            Tx=Y; Ty=-X; T0=(Tpwm-Tx-Ty)/2;
            break;
        case 3://扇区1
            Tx=-Z; Ty=X; T0=(Tpwm-Tx-Ty)/2;
            break;
        case 4://扇区4
            Tx=-X; Ty=Z; T0=(Tpwm-Tx-Ty)/2;
            break;
        case 5://扇区3
            Tx=X; Ty=-Y; T0=(Tpwm-Tx-Ty)/2;
            break;
        case 6://扇区5
            Tx=-Y; Ty=-Z; T0=(Tpwm-Tx-Ty)/2;
            break;
        default:
            Tx = 0.0f; Ty = 0.0f;
            break;
    }
    // 过调制
    if( (Tx+Ty) > Tpwm )
    {
        float Tsum = Tx + Ty;
        Tx=Tpwm * Tx / Tsum;
        Ty=Tpwm * Ty / Tsum;
    }
    /*    扇区切换时间点确定:CCR    */
    Ta=(Tpwm-Tx-Ty)/4;
    Tb=Ta+Tx/2;
    Tc=Tb+Ty/2;
    switch(N)
    {
        case 1://扇区2
            TIM1->CCR1=Tb; TIM1->CCR2=Ta; TIM1->CCR3=Tc;
            break;
        case 2://扇区6
            TIM1->CCR1=Ta; TIM1->CCR2=Tc; TIM1->CCR3=Tb;
            break;
        case 3://扇区1
            TIM1->CCR1=Ta; TIM1->CCR2=Tb; TIM1->CCR3=Tc;
            break;
        case 4://扇区4
            TIM1->CCR1=Tc; TIM1->CCR2=Tb; TIM1->CCR3=Ta;
            break;
        case 5://扇区3
            TIM1->CCR1=Tc; TIM1->CCR2=Ta; TIM1->CCR3=Tb;
            break;
        case 6://扇区5
            TIM1->CCR1=Tb; TIM1->CCR2=Tc; TIM1->CCR3=Ta;
            break;
        default:
            TIM1->CCR1=(uint16_t)Ta; TIM1->CCR2=(uint16_t)Ta; TIM1->CCR3=(uint16_t)Ta;
            break;
    }
}

