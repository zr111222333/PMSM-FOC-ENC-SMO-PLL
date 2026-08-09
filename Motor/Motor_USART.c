#include "Motor_USART.h"


UART_HandleTypeDef huart1;

void Motor_USART_Init(void)
{
    // 1. 使能时钟
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    // 2. 配置 TX 和 RX 引脚
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1; // GPIO 引脚的复用功能编号，告诉单片机把这个引脚连接到 USART1 外设上
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pin = USART_TX_GPIO_PIN;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(USART_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = USART_RX_GPIO_PIN;
    HAL_GPIO_Init(USART_RX_GPIO_PORT, &GPIO_InitStruct);

    // 配置 UART 参数
    huart1.Instance = USART1;                        // 把USART1外设的基地址强转成 USART_TypeDef * 指针
    huart1.Init.BaudRate = 115200;                   // 波特率
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;     // 无硬件流控
    huart1.Init.Mode = UART_MODE_TX_RX;              // 收发模式
    huart1.Init.OverSampling = UART_OVERSAMPLING_16; // 每位采样16次，抗噪声能力大
    huart1.Init.Parity = UART_PARITY_NONE;           // 无校验
    huart1.Init.StopBits = UART_STOPBITS_1;          // 1个停止位
    huart1.Init.WordLength = UART_WORDLENGTH_8B;     // 8位数据
    HAL_UART_Init(&huart1);

    // 电机控制项目中，通常不需要开启串口接收中断
}


/**
 * @brief  重定向 fputc，使 printf 可以通过串口输出
 */
// int fputc(int ch, FILE *f)
// {
//     // 使用 HAL 库阻塞发送，超时时间设为 0xFFFF
//     HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, 0xFFFF);
//     return ch;
// }
