#include "PID.h"

PID_Struct g_speed_pid;
PID_Struct g_current_pid;
PID_Struct g_current_pid_Id;
PID_Struct g_current_pid_Iq;
volatile float g_current_Kp = I_Kp_defult;
volatile float g_current_Ki = I_Ki_default;


void PID_Init(PID_Struct *pid,float Kp,float Ki,float Kd,float Kb,float OutMax,float OutMin)
{
    pid->Kp=Kp;
    pid->Ki=Ki;
    pid->Kd=Kd;
    pid->Kb=Kb;
    pid->Error=0.0f;
    pid->Error_Last=0.0f;
    pid->Error_Sum=0.0f;
    pid->SetPoint=0.0f;
    pid->Output=0.0f;
    pid->OutMax=OutMax;
    pid->OutMin=OutMin;
    pid->Output_raw = 0.0f;
}


/**
 * @brief 位置式 PID + 反计算抗饱和
 * @param PID 结构体指针
 * @param Feedback_value 当前反馈值
 * @retval float 形式的输出（浮点结果存于 PID->Output）
 */
float PID_Ctrl(PID_Struct *pid, float Feedback_value)
{
    pid->Error = pid->SetPoint - Feedback_value;
    pid->Error_Sum = pid->Error * pid->Ki+ pid->Error_Sum;

    pid->Output_raw = pid->Kp * pid->Error 
                + pid->Error_Sum 
                + pid->Kd * (pid->Error-pid->Error_Last);
    /* 结果输出限幅  */
    if(pid->Output_raw > pid->OutMax)
        pid->Output=pid->OutMax;
    else if(pid->Output_raw < pid->OutMin)
        pid->Output=pid->OutMin;
    else
        pid->Output = pid->Output_raw;// 没超限就直接等于未饱和值

    /*  误差和输出限幅 ： 反计算抗饱和
         如果没有超限，pid->Output == Output_Unsat，后面这项为0，不干预。
         如果超限了，(pid->Output - Output_Unsat) 是负数，会把 Error_Sum 往回拉
    */
    /* 输出差和积分状态现在单位一致 */
    pid->Error_Sum = pid->Error_Sum + pid->Kb * (pid->Output - pid->Output_raw);
    
    pid->Error_Last = pid->Error;
    
    return pid->Output;//可以不要，计算完直接赋给地址参数里了
}




