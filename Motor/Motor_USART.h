#ifndef __MOTOR_USART_H
#define __MOTOR_USART_H

#include "main.h"

extern UART_HandleTypeDef huart1;

#define USART_TX_GPIO_PORT GPIOB
#define USART_TX_GPIO_PIN  GPIO_PIN_6

#define USART_RX_GPIO_PORT GPIOB
#define USART_RX_GPIO_PIN  GPIO_PIN_7


void Motor_USART_Init(void);    //UART≥ı ºªØ≈‰÷√

#endif
