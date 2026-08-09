// #include "./SYSTEM/sys/sys.h"
// #include "./SYSTEM/usart/usart.h"
// #include "./SYSTEM/delay/delay.h"
// #include "./BSP/LED/led.h"
// #include "./BSP/KEY/key.h"
#include "./main.h"
// #include "Motor_PID_Tuner.h"
int main(void)
{     


    HAL_Init();                              /* 初始化HAL库 */
    sys_stm32_clock_init(336, 8, 2, 7);      /* 设置时钟,168Mhz */
    delay_init(168);                         /* 延时初始化 */
    usart_init(115200);                      /* 串口初始化为115200 */
    led_init();                              /* 初始化LED */
    key_init();                              /* 初始化按键 */
    // Motor_USART_Init();                   
    // Motor_Key_Init();
    
    /* 1. 全部硬件初始化，不启动 */
    PMSM_TIM_Init();             //TIM1三相PWM、死区、ADC触发通道
    Motor_ADC_Init();           // ADC1/2/3、DMA、中断
    Encoder_Init();            // TIM3编码器接口
    Motor_CAN_Init();          // CAN1 500kbps
    SpeedLoop_TIM6_Init();    // TIM6:1ms定时器
    // PMSM_Init();

    /* 2. 全部软件初始化 */
    PID_Init(&g_current_pid_Id,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
    PID_Init(&g_current_pid_Iq,I_Kp, I_Ki, I_Kd, I_Kb,I_OutMax, I_OutMin);
    PID_Init(&g_speed_pid,S_Kp, S_Ki, S_Kd, S_Kb,S_OutMax, S_OutMin);
    Foc_Init();         // 初始化Id/Iq给定、母线电压等FOC变量
    SMO_Init();
    Motor_Protect_Init();     // 初始化过流、过压、欠压等保护参数
    Motor_Identify_Init();    // 初始化参数辨识状态机
    Motor_StateInit();        // 状态机进入IDLE，清零状态变量


    /* 3. 启动后台采样和周期任务 */
    /* 
        TIM6可以在 main 中统一启动；
        TIM1涉及电机功率输出，不能直接统一启动，应由状态机在零偏、对齐和运行阶段分别控制 
    */
    Motor_ADC_Start();        // ADC进入等待，等TIM1_CH4触发采样
    Encoder_Start();  
    Motor_CAN_Start();
    SpeedLoop_TIM6_Start();   // 启动1ms编码器更新、200Hz速度环和保护任务
    
    /* 4. 启动状态机 */
    Motor_SetState(motor_state_idle);
    uint32_t state_task_tick = HAL_GetTick();//控制状态任务每约1ms执行一次
    uint32_t vofa_output_tick = state_task_tick;//控制串口每约10ms输出一次，也就是约100Hz

    //初始化自动调参接口
    // Motor_PID_Tuner_Init();

    while (1)
     {
        uint8_t key_value = key_scan(0);
        float speed = 0;

        if(key_value == 1)         /* KEY0按下 */
        {
            if(Motor_GetState() == motor_state_idle )
            {
                Motor_SetState(motor_state_current_offset);//进入电流零偏校准
                Motor_CAN_NotifyLocalControl();//通知CAN主控本地控制
            } 
        }
        else if(key_value == 2)         /* KEY1按下 */
        {
            if(Motor_GetState() == motor_state_encoder_run || (Motor_GetState() == motor_state_smo_run))
            {
                speed = g_speed_ref_rpm + 50;
                if(speed >= 2700)
                    speed = 2700;
                else if(speed <= -2700)
                    speed = -2700;
                Motor_SetSpeedRef(speed);
                Motor_CAN_NotifyLocalControl();
            }
                
        }
        else if(key_value == 3)         /* KEY2按下 */
        {
            if(Motor_GetState() == motor_state_encoder_run || (Motor_GetState() == motor_state_smo_run))
            {
                speed = g_speed_ref_rpm - 50;
                if(speed >= 2700)
                    speed = 2700;
                else if(speed <= -2700)
                    speed = -2700;
                Motor_SetSpeedRef(speed);
                Motor_CAN_NotifyLocalControl();
            }

        }

        uint32_t now_tick = HAL_GetTick();
        /* 状态任务由主循环按约1ms调度，切换稳定时间内部使用HAL tick。 */
        if (now_tick - state_task_tick >= 1U)
        {
            state_task_tick = now_tick;
            Motor_CAN_Task_1ms();  //处理CAN接收队列、发送状态帧、心跳帧、超时停止
            Motor_StateTask_1ms();//处理状态机、FOC开环、闭环切换、参数辨识、保护等
        }

        /*
         * VOFA FireWater：仅主循环约100Hz输出，禁止在10kHz中断中printf。
         * 状态值：0空闲、1零偏、2对齐、3参数辨识、4开环、5编码器、6 SMO、7故障。
         */
        if (now_tick - vofa_output_tick >= 10)
        {
            float encoder_theta;
            float smo_theta;
            float encoder_speed_rpm;
            float smo_speed_rpm;
            float angle_error;
            uint8_t angle_valid;
            Motor_State state_snapshot;

            encoder_theta = g_encoder_elec_angle;
            smo_theta = SMO_Output.PLL_theta;
            encoder_speed_rpm = g_motor_speed;
            smo_speed_rpm = SMO_Output.speed_rpm;
            angle_error = g_encoder_smo_angle_error; /* encoder_theta - smo_theta，[-pi,pi] */
            angle_valid = SMO_Output.angle_valid;
            state_snapshot = Motor_GetState();

            vofa_output_tick = now_tick;


            printf("%.6f,%.6f,%.3f,%.3f,%.6f,%u,%u\r\n",
                   encoder_theta,
                   smo_theta,
                   encoder_speed_rpm,
                   smo_speed_rpm,
                   angle_error,
                   (unsigned int)angle_valid,
                   (unsigned int)state_snapshot);
            //马鞍波       
            // float duty_u = (float)TIM1->CCR1 / MOTOR_PWM_PERIOD;
            // float duty_v = (float)TIM1->CCR2 / MOTOR_PWM_PERIOD;
            // float duty_w = (float)TIM1->CCR3 / MOTOR_PWM_PERIOD;
            // printf("%.6f,%.6f,%.6f\r\n",
            //             duty_u,
            //             duty_v,
            //             duty_w);    


        /* 接收AI下发的PID，并发送8列运行数据 */
        // Motor_PID_Tuner_Task();

        }
        
    }
}
