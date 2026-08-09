/*
 * CAN通信整体分为两部分：
 *
 * 一、正常执行CAN通信
 *
 * Motor_CAN_Init()
 *      ↓
 * 初始化CAN引脚、500kbps波特率、过滤器和中断
 *      ↓
 * Motor_CAN_Start()
 *      ↓
 * 启动CAN，开启接收和错误通知
 *      ↓
 * 上位机发送控制帧或速度帧
 *      ↓
 * CAN接收中断
 *      ↓
 * 只把数据放入接收队列，不在中断中处理命令
 *      ↓
 * Motor_CAN_Task_1ms()
 *      ↓
 * 从队列取出数据，判断控制命令或速度命令
 *      ↓
 * 调用电机状态、控制模式和速度给定接口
 *      ↓
 * 返回命令响应、运行状态和心跳数据
 *
 *
 * 二、数据判断与通信保护
 *
 * 接收到CAN数据
 *      ↓
 * 检查帧类型、ID、数据长度和接收队列
 *      ↓
 * 检查命令、参数、目标速度和电机当前状态
 *      ↓
 * 数据正确：执行命令并返回成功响应
 * 数据错误：拒绝执行并返回失败原因
 *      ↓
 * CAN控制超时：目标速度清零，电机减速停止
 * CAN出现Bus-Off：停止电机，清空队列，延时重新启动CAN
 * 本地按键控制：取消CAN控制权，避免CAN超时误停机
 */

#include "Motor_CAN.h"

CAN_HandleTypeDef hcan1;
//CAN周期和队列大小
#define CAN_RX_QUEUE_SIZE       4U      /*  接收队列最多保存4帧  */
#define CAN_STATUS_PERIOD_MS    20U     /*  每20ms发送一次状态  */ 
#define CAN_HEARTBEAT_PERIOD_MS 100U    /*  每100ms发送一次心跳 */
#define CAN_RECOVERY_DELAY_MS   1000U   /*  总线关闭每1秒尝试恢复*/
#define CAN_STOP_MAX_MS         8000U   /*  最大停止时间8000ms */
//开启这些通知：收到新CAN帧、CAN总线断开、CAN出现错误、CAN产生最后错误码
#define CAN_NOTIFICATIONS       (CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_BUSOFF | \
                                 CAN_IT_ERROR | CAN_IT_LAST_ERROR_CODE)

//接收帧结构体: 只接收两种帧，控制、速度帧
typedef struct 
{
    uint16_t id;
    uint8_t data[2];
} CAN_RxFrame;

static volatile CAN_RxFrame s_rx_queue[CAN_RX_QUEUE_SIZE];//接收队列
static volatile uint8_t     s_rx_head, s_rx_tail;//头中断写入的位置，尾1ms任务读取的位置
static volatile uint8_t s_can_available;        //CAN当前能否正常收发
static volatile uint8_t s_bus_off_pending;      //CAN总线关闭标志，等待主任务处理
static volatile uint8_t s_recovering;           //正在尝试恢复CAN
static volatile Motor_CAN_Stats s_stats;        //CAN状态错误统计
static uint8_t s_init_ok, s_can_owns_run, s_timeout_stopping;//CAN初始化成功标志，CAN是否拥有电机运行控制权标志，CAN控制超时停止标志
static uint32_t s_last_control_tick, s_timeout_tick, s_recovery_tick;//上次收到控制命令的时间戳，CAN控制超时停止的时间戳，CAN总线关闭恢复的时间戳
static uint32_t s_status_tick, s_heartbeat_tick, s_control_count;//上次发送状态帧的时间戳，上次发送心跳帧的时间戳，收到控制命令的累计次数


/* 数据打包函数：将16位整数拆分为两个字节 
*   采用的是高字节在前，也就是大端格式
*/ 
static void PutU16(uint8_t *p, uint16_t value)
{
    p[0] = (uint8_t)(value >> 8);// 原value的高8位
    p[1] = (uint8_t)value;      // 原value的低8位
}

/* 数据打包函数：将32位整数拆分为两个16位数据 
*   采用的是高字节在前，也就是大端格式
*/ 
static void PutU32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24); p[1] = (uint8_t)(value >> 16);
    p[2] = (uint8_t)(value >> 8);  p[3] = (uint8_t)value;
}

/* 将浮点数缩放为16位有符号整数，方便放进 CAN 数据帧发送
*   @brief： CAN 一帧最多只有 8 字节。直接发送一个 float 要占 4 字节，而且不同设备还可能存在字节序问题。
*           缩放成 int16_t 后只占 2 字节，更适合 CAN
*/
static int16_t ScaleS16(float value, float gain)
{
    float scaled;
    if (isnan(value) || isinf(value)) return 0;
    scaled = value * gain;
    if (scaled > 32767.0f) return 32767;//限幅int16_t
    if (scaled < -32768.0f) return -32768;
    return (int16_t)(scaled + ((scaled >= 0.0f) ? 0.5f : -0.5f));//进行四舍五入：正数加 0.5，负数减 0.5
}

/* 将浮点数缩放为16位无符号整数，方便放进 CAN 数据帧发送 */ 
static uint16_t ScaleU16(float value, float gain)
{
    float scaled;
    if (isnan(value) || isinf(value) || (value <= 0.0f)) 
        return 0U;
    scaled = value * gain;
    return (scaled > 65535.0f) ? 65535U : (uint16_t)(scaled + 0.5f);
}


/** 
 * @brief 通用发送 CAN 发送帧函数：不用每发送一帧，都重新配置一次
 *  @param id 帧 ID
 *  @param data 数据指针
 *  @param dlc 数据长度
 *  @return 1 表示发送成功，0 表示发送失败
 */
static uint8_t SendFrame(uint16_t id, uint8_t *data, uint8_t dlc)
{
    CAN_TxHeaderTypeDef tx = {0};//创建 CAN 发送帧头
    uint32_t mailbox;            //保存使用了哪个发送邮箱号
    if (s_can_available == 0U) 
        return 0U;
    tx.StdId = id;          //标准帧ID
    tx.IDE = CAN_ID_STD;    //标准帧
    tx.RTR = CAN_RTR_DATA;  //数据帧：我要发送实际数据
    tx.DLC = dlc;           //数据长度
    tx.TransmitGlobalTime = DISABLE;//不使用时间触发功能，不发送时间戳
    /* HAL_CAN_AddTxMessage() 会自动寻找空闲邮箱，并把实际使用的邮箱编号写入 mailbox */
    if (HAL_CAN_AddTxMessage(&hcan1, &tx, data, &mailbox) == HAL_OK) //判断 CAN 当前是否可用，检查有没有空闲发送邮箱
        return 1U;//发送成功
    s_stats.tx_failed_count++;//发送失败累计次数
    s_stats.last_error_code = HAL_CAN_GetError(&hcan1);//从 HAL 库读取当前 CAN 错误码
    return 0U;//发送失败
}

/** 
 * @brief 清空接收队列旧数据
 */
static void ClearRxQueue(void)
{
    // uint32_t primask = __get_PRIMASK(); //获取当前中断状态，0表示中断使能，1表示中断禁止
    // __disable_irq();                    //禁止中断，防止接收中断修改队列数据
    // s_rx_tail = s_rx_head;              /* 读指针追上写指针，逻辑上清空接收队列 */
    // if (primask == 0U)                  //如果之前中断是使能状态，则恢复中断
    //     __enable_irq();

    uint32_t irq_enabled;
    /* 保存CAN接收中断原来的开启状态 */
    irq_enabled = NVIC_GetEnableIRQ(CAN1_RX0_IRQn);
    /* 暂时关闭CAN接收中断 */
    HAL_NVIC_DisableIRQ(CAN1_RX0_IRQn);
    /* 读指针追上写指针，清空接收队列，接收即读 */
    s_rx_tail = s_rx_head;
    /* 原来开启才恢复，原来关闭则继续保持关闭 */
    if (irq_enabled != 0U)
        HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

/** 
 * @brief 从接收队列中取出一帧数据存起来：接收与处理分离
 *  @param frame 接收帧指针
 *  @return 1 表示取到数据，0 表示队列为空
 */
static uint8_t PopRx(CAN_RxFrame *frame)
{
    uint8_t tail = s_rx_tail;       //保存当前读指针位置
    if (tail == s_rx_head)          //判断队列是否为空，头尾相等为空
        return 0U;
    //把当前队列位置的一帧数据复制到调用者提供的 frame
    frame->id = s_rx_queue[tail].id;
    frame->data[0] = s_rx_queue[tail].data[0];
    frame->data[1] = s_rx_queue[tail].data[1];
    __DMB();
    s_rx_tail = (uint8_t)((tail + 1U) % CAN_RX_QUEUE_SIZE);//读取位置向后移动
    return 1U;
}

/** 
 *  @brief 标记接收到控制命令
 *  @param now 当前时间戳
 */
static void MarkControl(uint32_t now)
{
    s_control_count++;          //有效控制命令计数加一
    s_last_control_tick = now; //本次控制命令到达的时间
    s_timeout_stopping = 0U;  //收到控制命令，清除超时停止标志
}

static void StopMotor(void)
{
    if (Motor_GetState() == motor_state_identify)//如果正在参数辨识，先中止辨识
        Motor_Identify_Abort(motor_identify_fail_not_stopped);
    Motor_SetSpeedRef(0.0f); 
    Motor_SetState(motor_state_idle);
}

/** 
 * @brief 启动外设
 * @return 1 表示启动成功，0 表示启动失败
 */
static uint8_t StartPeripheral(void)
{
    //启动 CAN 外设
    if (HAL_CAN_Start(&hcan1) != HAL_OK) 
        return 0U;
    //开启 CAN 中断通知（HAL_CAN_ActivateNotification）
    if (HAL_CAN_ActivateNotification(&hcan1, CAN_NOTIFICATIONS) == HAL_OK) 
        return 1U;
    (void)HAL_CAN_Stop(&hcan1);//如果开启中断失败，则停止 CAN 外设
    return 0U;
}


void Motor_CAN_Init(void)
{
    GPIO_InitTypeDef  gpio = {0};
    // CAN_InitTypeDef   当这个 CAN 节点只负责发送、不需要接收报文时，可以只进行 CAN 基础初始化
    CAN_FilterTypeDef filter = {0};//接收时才需要筛选器

    MOTOR_CAN_RX_GPIO_CLK_ENABLE();
    MOTOR_CAN_TX_GPIO_CLK_ENABLE();
    __HAL_RCC_CAN1_CLK_ENABLE();
    
    gpio.Alternate=MOTOR_CAN_GPIO_AF;
    gpio.Mode=GPIO_MODE_AF_PP;  //1.can通信要高低电平  2.引脚信号进入TJA1050靠这个芯片控制非手动控制
    gpio.Pin=MOTOR_CAN_RX_PIN;
    gpio.Pull=GPIO_NOPULL;
    gpio.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
    HAL_GPIO_Init(MOTOR_CAN_RX_PORT, &gpio);
    gpio.Pin=MOTOR_CAN_TX_PIN;
    HAL_GPIO_Init(MOTOR_CAN_TX_PORT, &gpio);

    hcan1.Instance=CAN1;
    hcan1.Init.AutoBusOff=DISABLE;          //使用自己编写的恢复流程
    hcan1.Init.AutoRetransmission=DISABLE;  //关闭自动重发，防止占住邮箱
    hcan1.Init.AutoWakeUp=ENABLE;           //：一旦监测到 CAN 消息，即通过硬件自动退出睡眠模式。
    hcan1.Init.Mode=CAN_MODE_NORMAL;
    hcan1.Init.Prescaler=6;  /* 42MHz/(6*14)=500kbps, 发送“1个二进制位”需要14个TQ,500kbps 很合适,线路很短、数据量不大*/
    hcan1.Init.ReceiveFifoLocked=DISABLE;  //新帧可能覆盖FIFO中尚未读取的旧帧
    hcan1.Init.SyncJumpWidth=CAN_SJW_1TQ;  //总线上两个设备的晶振不可能完全一样，每次同步时，最多允许把采样位置修正1个TQ
    hcan1.Init.TimeSeg1=CAN_BS1_11TQ;      //采样点从一个CAN位开始，到采样点之前，共经过11个TQ
    hcan1.Init.TimeSeg2=CAN_BS2_2TQ;       //14-1-11=2采样完成后，到当前 CAN 位结束，还剩 2 个 TQ
    hcan1.Init.TimeTriggeredMode=DISABLE;//关闭时间触发通信功能,开即所有节点严格按照统一时间表发送
    hcan1.Init.TransmitFifoPriority=DISABLE;//DISABLE，按照 CAN ID 消息标识符优先级发送
    if (HAL_CAN_Init(&hcan1) != HAL_OK) //先初始化再判断
    {
        s_stats.last_error_code = HAL_CAN_GetError(&hcan1); return;
    }

    filter.FilterActivation=ENABLE;              //使能筛选器
    filter.FilterBank=0;                         //使用第0个筛选器，只有使用了2个ID，一个过滤器组最多能精确匹配 4 个标准 ID
    filter.FilterFIFOAssignment=CAN_FILTER_FIFO0;//把接收的帧放入FIFO0
    filter.FilterIdHigh=(uint32_t)(MOTOR_CAN_ID_CONTROL << 5);//左移5位，放到高位，标准ID必须放在位15～5，低5位留给 RTR、IDE 等控制位
    filter.FilterIdLow=(uint32_t)(MOTOR_CAN_ID_SPEED << 5);
    filter.FilterMaskIdHigh=filter.FilterIdHigh; //左移5位，放到高位
    filter.FilterMaskIdLow=filter.FilterIdLow;
    filter.FilterMode=CAN_FILTERMODE_IDMASK;     //使用掩码模式，允许多个ID通过
    filter.FilterScale=CAN_FILTERSCALE_16BIT;    //16位掩码模式
    filter.SlaveStartFilterBank=14;              //单CAN模式，14个滤波器都分配给主CAN
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) 
    {
        s_stats.last_error_code = HAL_CAN_GetError(&hcan1); return;
    }

    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 3U, 0U);//CAN接收中断优先级比总线错误中断高，避免总线错误中断打断接收中断
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);          //开启CAN接收中断
    HAL_NVIC_SetPriority(CAN1_SCE_IRQn, 3U, 1U);//CAN总线错误中断优先级比接收中断低，避免总线错误中断打断接收中断
    HAL_NVIC_EnableIRQ(CAN1_SCE_IRQn);          //开启CAN总线 错误中断
    s_init_ok = 1;                              //标记CAN初始化成功

}


/** 
 * @brief 启动CAN外设
 *  @note  1. 只有在CAN初始化成功后，才能调用此函数启动CAN外设
 *         2. 如果CAN总线关闭，则会尝试恢复CAN总线
 *         3. 时间戳：用于超时判断和周期发送 设置一个统一的计时起点
 */
void Motor_CAN_Start(void)
{
    uint32_t now = HAL_GetTick();
    if ((s_init_ok != 0U) && (StartPeripheral() != 0U)) //不是只做判断，而是先真正执行这个函数，再用函数返回值进行判断
    {
        s_can_available = 1U; //CAN可用
        s_recovering = 0U;    //CAN总线关闭恢复 标志清零
    } 
    else if (s_init_ok != 0U) 
    {
        s_recovering = 1U; 
        s_recovery_tick = now;//标记CAN总线关闭恢复的时间戳
        s_stats.last_error_code = HAL_CAN_GetError(&hcan1);
    }
    //标记上次收到控制命令、发送状态、发送心跳的时间戳
    s_last_control_tick = s_status_tick = s_heartbeat_tick = now;
}
    

/* 电机处理完命令后，给上位机返回的回执 */
void Motor_CAN_SendResponse(uint8_t command, uint8_t result, uint8_t reason)
{
    uint8_t data[3] = {command, result, reason};
    (void)SendFrame(MOTOR_CAN_ID_RESPONSE, data, 3U);
}


/* @brief 发送电机状态、运行时数据、参数辨识结果
 * @note 1. 状态帧：速度参考值、实际速度值、电机状态、控制模式、保护故障值、CAN可用标志、CAN拥有运行控制权标志、CAN控制超时停止标志、SMO角度有效标志
 *       2. 运行时帧：电流d轴、电流q轴、母线电压、堵转保护需要的速度斜坡
 *       3. 参数辨识结果帧：定子电阻Rs、定子电感Ls、参数辨识状态、参数辨识失败原因
 */
void Motor_CAN_SendStatus(void)
{
    uint8_t data[8] = {0};//每个 data[i] 是 1 字节，也就是 8 位
    Motor_State state = Motor_GetState();
    float speed = (state == motor_state_smo_run) ? SMO_Output.speed_rpm : g_motor_speed;

    //把速度参考值、实际速度值、电机状态、控制模式、保护故障值、CAN可用标志、CAN拥有运行控制权标志、CAN控制超时停止标志、SMO角度有效标志打包发送
    PutU16(&data[0], (uint16_t)ScaleS16(g_speed_ref_rpm, 1.0f));//把速度参考值
    PutU16(&data[2], (uint16_t)ScaleS16(speed, 1.0f));          //把实际速度值
    data[4] = (uint8_t)state; data[5] = (uint8_t)Motor_GetControlMode();//把电机状态和控制模式
    data[6] = (uint8_t)g_protect.Fault;//把保护故障值
    //把CAN可用、CAN拥有运行控制权、CAN控制超时停止、SMO角度有效标志打包
    data[7] = (uint8_t)(s_can_available | (s_can_owns_run << 1) |
                       (s_timeout_stopping << 2) | (SMO_Output.angle_valid << 3));
    (void)SendFrame(MOTOR_CAN_ID_STATUS, data, 8U);//发送状态帧
    
    //把电流、母线电压、堵转保护需要的速度斜坡、参数辨识结果打包发送：float -> int16_t/uint16_t
    PutU16(&data[0], (uint16_t)ScaleS16(I_dq.Id, 100.0f));
    PutU16(&data[2], (uint16_t)ScaleS16(I_dq.Iq, 100.0f));
    PutU16(&data[4], ScaleU16(foc_input.Udc, 100.0f));
    PutU16(&data[6], (uint16_t)ScaleS16(g_speed_ref_now_rpm, 1.0f));
    (void)SendFrame(MOTOR_CAN_ID_RUNTIME, data, 8U);//发送运行时帧
    
    //把参数辨识结果打包发送：float -> uint16_t
    PutU16(&data[0], ScaleU16(g_motor_identify.Rs, 1000.0f));
    PutU16(&data[2], ScaleU16(g_motor_identify.Ls, 1000000.0f));
    data[4] = (uint8_t)g_motor_identify.State;
    data[5] = (uint8_t)g_motor_identify.Fail; 
    data[6] = data[7] = 0U;
    (void)SendFrame(MOTOR_CAN_ID_PARAMETER, data, 8U);
}

/* @brief 发送心跳帧：还在线，并顺便上报当前基本状态
 * @note 1. 心跳帧：固定标志位0xA5、电机状态、保护故障值、CAN可用标志、CAN拥有运行控制权标志、CAN控制超时停止标志
 */
void Motor_CAN_SendHeartbeat(void)
{
    uint8_t data[8];
    data[0] = 0xA5U; 
    data[1] = (uint8_t)Motor_GetState();
    data[2] = (uint8_t)g_protect.Fault; 
    data[3] = s_can_available;
    PutU32(&data[4], s_control_count);
    (void)SendFrame(MOTOR_CAN_ID_HEARTBEAT, data, 8U);
}


/* @brief MCU处理速度命令：上位机发送的速度命令
 * @param msb 速度高字节
 * @param lsb 速度低字节
 * @param now 当前时间戳
 */
static void ProcessSpeed(uint8_t msb, uint8_t lsb, uint32_t now)
{
    int16_t speed = (int16_t)(((uint16_t)msb << 8) | lsb);
    Motor_State state = Motor_GetState();
    if ((speed < -MOTOR_CAN_MAX_SPEED_RPM) || (speed > MOTOR_CAN_MAX_SPEED_RPM)) 
    {
        //速度超出范围，返回失败响应
        Motor_CAN_SendResponse(MOTOR_CAN_CMD_SET_SPEED, MOTOR_CAN_RESULT_FAILED,
                               MOTOR_CAN_REASON_OUT_OF_RANGE);
    } 
    //如果电机处于故障状态，返回失败响应
    else if ((g_protect.Fault != Fault_NONE) || (state == motor_state_fault)) 
    {
        Motor_CAN_SendResponse(MOTOR_CAN_CMD_SET_SPEED, MOTOR_CAN_RESULT_FAILED,
                               MOTOR_CAN_REASON_FAULT_ACTIVE);
    } 
    //速度正常执行速度
    else 
    {
        if (speed != 0) 
            Motor_SetDirection((speed < 0) ? CCW : CW);
        Motor_SetSpeedRef((float)speed); 
        MarkControl(now);
        Motor_CAN_SendResponse(MOTOR_CAN_CMD_SET_SPEED, MOTOR_CAN_RESULT_OK,MOTOR_CAN_REASON_NONE);
    }
}


/** 
 * @brief MCU处理控制命令：上位机发送的启动、停止、模式切换等命令
 * @param command 控制命令
 * @param parameter 控制参数
 * @param now 当前时间戳
 * @brief 读取当前电机状态->检查故障状态->检查帧参数是否合法->根据 command 进入不同 case
 *                  ->检查故障和状态条件->调用电机状态机接口->发送成功或失败响应
 */
static void ProcessControl(uint8_t command, uint8_t parameter, uint32_t now)
{
    Motor_State state = Motor_GetState();
    //如果不是设置模式命令，且参数不为0，则返回帧格式错误响应
    if ((command != MOTOR_CAN_CMD_SET_MODE) && (parameter != 0U)) {
        Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED, MOTOR_CAN_REASON_FRAME_FORMAT); 
        return;
    }
    //根据不同的控制命令，执行不同的操作
    switch (command) {
    case MOTOR_CAN_CMD_START://如果是启动命令，检查电机状态和故障状态
        if (g_protect.Fault != Fault_NONE)//如果电机处于故障状态，返回失败响应
            Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED, MOTOR_CAN_REASON_FAULT_ACTIVE);
        else if (state != motor_state_idle)//如果电机不处于停机状态，禁止重复启动
            Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED, MOTOR_CAN_REASON_STATE_DENIED);
        else {                             //如果电机处于停机状态，且没有故障，则启动电机
            Motor_SetState(motor_state_current_offset);
            //如果电机不处于电流零偏校准状态，返回失败响应
            if (Motor_GetState() != motor_state_current_offset) {
                Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED,MOTOR_CAN_REASON_STATE_DENIED); 
                break;
            }
            s_can_owns_run = 1U;     //标记CAN拥有电机运行控制权
            MarkControl(now);       //标记上次收到控制命令的时间戳
            Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_OK, MOTOR_CAN_REASON_NONE);
        }
        break;
    case MOTOR_CAN_CMD_STOP:
        StopMotor();
        s_can_owns_run = 0U; 
        MarkControl(now);
        Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_OK, MOTOR_CAN_REASON_NONE);
        break;
    //如果是设置模式命令，且参数不为0，则执行模式切换
    case MOTOR_CAN_CMD_SET_MODE:
        if ((g_protect.Fault != Fault_NONE) || (state == motor_state_fault))
            Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED, MOTOR_CAN_REASON_FAULT_ACTIVE);
        //如果参数超出范围，返回失败响应
        else if (parameter > (uint8_t)motor_control_smo)
            Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED, MOTOR_CAN_REASON_MODE_INVALID);
        //如果模式切换失败，返回失败响应
        else if (Motor_SetControlMode((Motor_ControlMode)parameter) == 0U)
            Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED, MOTOR_CAN_REASON_STATE_DENIED);
        //如果模式切换成功，返回成功响应
        else {
            MarkControl(now);
            Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_OK, MOTOR_CAN_REASON_NONE);
        }
        break;
    //如果是状态请求命令，发送状态帧
    case MOTOR_CAN_CMD_STATUS_REQUEST:
        Motor_CAN_SendStatus(); 
        break;
    case MOTOR_CAN_CMD_CLEAR:
        if (s_can_owns_run != 0U)
            StopMotor();
        s_can_owns_run = 0U; 
        s_timeout_stopping = 0U; 
        ClearRxQueue();
        break;
    default:
        Motor_CAN_SendResponse(command, MOTOR_CAN_RESULT_FAILED, MOTOR_CAN_REASON_FRAME_FORMAT); 
        break;
    }
}


/** 
 * @brief CAN超时停止：如果上位机长时间没有发送控制命令，则停止电机
 * @param now 当前时间戳
 */
static void HandleTimeout(uint32_t now)
{
    Motor_State state;
    if (s_can_owns_run == 0U) return;//如果CAN没有拥有电机运行控制权，则不需要处理超时停止
    //超时标志没有使但时间戳超过了超时阈值，则停止电机
    if ((s_timeout_stopping == 0U) && ((uint32_t)(now - s_last_control_tick) > MOTOR_CAN_TIMEOUT_MS)) {
        Motor_SetSpeedRef(0.0f); //先把速度参考值清零，防止电机继续运行
        s_timeout_stopping = 1U; 
        s_timeout_tick = now;
    }
    if (s_timeout_stopping == 0U) return;//如果没有超时停止标志，则不需要处理
    state = Motor_GetState();
    //如果电机处于有感FOC或无感FOC状态，且速度参考值大于0.5rpm或实际速度大于30rpm，则不停止电机
    if ((state == motor_state_encoder_run) || (state == motor_state_smo_run)) {
        if (((fabsf(g_speed_ref_now_rpm) > 0.5f) || (fabsf(g_motor_speed) > 30.0f)) &&
            ((uint32_t)(now - s_timeout_tick) < CAN_STOP_MAX_MS)) 
                return;
    }
    StopMotor();
    s_can_owns_run = 0U; 
    s_timeout_stopping = 0U;
}


/** 
 * @brief 1ms任务：处理接收队列中的CAN帧，发送状态、心跳帧
 * @note 1. 如果CAN总线关闭，则尝试恢复CAN总线
 *       2. 如果接收队列中有数据，则处理速度命令或控制命令
 *       3. 如果接收队列中没有数据，则发送状态帧或心跳帧
 */
void Motor_CAN_Task_1ms(void)
{
    uint32_t now = HAL_GetTick();
    CAN_RxFrame frame;
    uint8_t processed = 0U;     //标记是否处理了接收队列中的数据
    //如果CAN总线关闭，则尝试恢复CAN总线
    if (s_bus_off_pending != 0U) 
    {   
        s_bus_off_pending = 0U; //清除总线关闭标志
        s_recovering = 1U;      //标记正在尝试恢复CAN总线
        s_recovery_tick = now;  //标记CAN总线关闭恢复的时间戳
        StopMotor();            //如果CAN总线关闭，通信已经严重异常，则停止电机
        s_can_owns_run = s_timeout_stopping = 0U; 
        ClearRxQueue();
    }
    //如果正在尝试恢复CAN总线，且已经超过恢复延迟时间，则尝试启动CAN外设
    if ((s_recovering != 0U) && ((uint32_t)(now - s_recovery_tick) >= CAN_RECOVERY_DELAY_MS)) 
    {
        s_recovery_tick = now; 
        (void)HAL_CAN_Stop(&hcan1);         //先停止CAN外设，再尝试启动CAN外设
        (void)HAL_CAN_ResetError(&hcan1);   //清除CAN错误码
        if (StartPeripheral() != 0U)        //如果启动CAN外设成功，则标记CAN可用，清除正在尝试恢复CAN总线标志，更新状态和心跳时间戳
        {
            s_can_available = 1U; 
            s_recovering = 0U;
            s_status_tick = s_heartbeat_tick = now;
        } 
        else 
            s_stats.last_error_code = HAL_CAN_GetError(&hcan1);
    }
    //如果CAN不可用，则不处理接收队列中的数据
    if (s_can_available == 0U) 
        return;
    //如果接收队列中有数据，则处理速度命令或控制命令
    if (PopRx(&frame) != 0U) {
        if (frame.id == MOTOR_CAN_ID_SPEED)
            ProcessSpeed(frame.data[0], frame.data[1], now);//速度LH数据需要合并为一个16位数据
        else ProcessControl(frame.data[0], frame.data[1], now);//控制命令和参数都在data[0]和data[1]中
        processed = 1U;
    }
    HandleTimeout(now);
    //如果接收队列中没有数据，则发送状态帧或心跳帧
    if ((processed == 0U) && ((uint32_t)(now - s_heartbeat_tick) >= CAN_HEARTBEAT_PERIOD_MS)) 
    {
        s_heartbeat_tick = now; 
        Motor_CAN_SendHeartbeat();//发送心跳帧
    } 
    else if ((processed == 0U) && ((uint32_t)(now - s_status_tick) >= CAN_STATUS_PERIOD_MS)) 
    {
        s_status_tick = now; 
        Motor_CAN_SendStatus();//发送状态帧
    }
}

/** 
 * @brief 上位机通知MCU，改变当前的控制权归属
 */
void Motor_CAN_NotifyLocalControl(void)
{
    s_can_owns_run = 0U;        //取消CAN拥有运行控制权标志
    s_timeout_stopping = 0U;    //取消CAN控制超时停止标志
}

/** 
 * @brief CAN 错误统计提供给外部查看
 * @return 返回CAN状态错误统计结构体指针
 */
const volatile Motor_CAN_Stats *Motor_CAN_GetStats(void)
{
    return &s_stats;
}

//CAN中断服务函数：接收中断
void CAN1_RX0_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }
//CAN总线错误中断
void CAN1_SCE_IRQHandler(void) { HAL_CAN_IRQHandler(&hcan1); }


/** 
 * @brief CAN接收中断回调函数：接收到新CAN帧时调用
 * @note 1. 如果不是CAN1，或者接收消息失败，则增加接收错误计数并返回
 *       2. 如果接收到的帧不是标准帧、数据帧、数据长度不为2，或者ID不是控制命令或速度命令，则增加接收错误计数并返回
 *       3. 如果接收队列满了，则增加接收错误计数并返回
 *       4. 否则，将接收到的帧保存到接收队列中，并更新写入位置
 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx;
    uint8_t data[8], next;
    //如果不是CAN1，或者接收消息失败，则增加接收错误计数并返回
    if ((hcan->Instance != CAN1) || (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx, data) != HAL_OK)) 
    {
        s_stats.rx_error_count++; 
        return;
    }
    //如果接收到的帧不是标准帧、数据帧、数据长度不为2，或者ID不是控制命令或速度命令，则增加接收错误计数并返回
    if ((rx.IDE != CAN_ID_STD) || (rx.RTR != CAN_RTR_DATA) || (rx.DLC != 2U) ||
        ((rx.StdId != MOTOR_CAN_ID_CONTROL) && (rx.StdId != MOTOR_CAN_ID_SPEED))) 
    {
        s_stats.rx_error_count++; 
        return;
    }
    next = (uint8_t)((s_rx_head + 1U) % CAN_RX_QUEUE_SIZE); //计算下一个写入位置
    //如果接收队列FIFO满了，则增加接收错误计数并返回
    if (next == s_rx_tail) 
    { 
        s_stats.rx_error_count++; 
        return; 
    }
    //这个数组的每个元素保存一帧经过筛选的 CAN 消息；每帧保存 ID 和两个有效数据字节
    s_rx_queue[s_rx_head].id = (uint16_t)rx.StdId;//保存接收到的帧ID
    s_rx_queue[s_rx_head].data[0] = data[0];      //保存接收到的帧数据
    s_rx_queue[s_rx_head].data[1] = data[1];      //保存接收到的帧数据
    __DMB();         //数据内存屏障，确保数据写入完成后再更新写入位置
    s_rx_head = next;//更新写入位置
}


/** 
 * @brief CAN总线错误中断回调函数：CAN总线出现错误时调用
 * @note 1. 如果不是CAN1，直接返回
 *       2. 获取当前CAN错误码，并保存到统计结构体中
 *       3. 如果接收FIFO溢出，则增加接收错误计数
 *       4. 如果发送失败，则增加发送失败计数
 *       5. 如果总线关闭，则标记CAN不可用，并增加总线关闭计数
 *       6. 重置CAN错误码
 */
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    uint32_t error;
    if (hcan->Instance != CAN1) 
        return;
    error = HAL_CAN_GetError(hcan); 
    s_stats.last_error_code = error;//保存当前CAN错误码
    if ((error & HAL_CAN_ERROR_RX_FOV0) != 0U) //接收FIFO0溢出
        s_stats.rx_error_count++;             //接收错误计数加一
    if ((error & (HAL_CAN_ERROR_ACK | HAL_CAN_ERROR_TX_TERR0 |
                  HAL_CAN_ERROR_TX_TERR1 | HAL_CAN_ERROR_TX_TERR2)) != 0U)//发送失败
        s_stats.tx_failed_count++;
    if ((error & HAL_CAN_ERROR_BOF) != 0U) {
        s_can_available = 0U;
        if ((s_bus_off_pending == 0U) && (s_recovering == 0U)) {
            s_stats.bus_off_count++; s_bus_off_pending = 1U;
        }
    }
    (void)HAL_CAN_ResetError(hcan);//清除CAN错误码
}

