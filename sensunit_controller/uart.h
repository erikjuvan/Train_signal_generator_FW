#pragma once

#include "stm32f7xx_hal.h"

#define UART_DEVICE_ID	1

#ifndef UART_DEVICE_ID
#error "UART_DEVICE_ID not defined"
#endif //UART_DEVICE_ID


#define UART_FRAME_START	0xFFFF

#define UART_BUFFER_SIZE	1024

#define USARTx                           USART3
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART3_CLK_ENABLE();
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOD_CLK_ENABLE() 

#define USARTx_FORCE_RESET()             __HAL_RCC_USARTD_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USARTD_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_8
#define USARTx_TX_GPIO_PORT              GPIOD
#define USARTx_TX_AF                     GPIO_AF7_USART3
#define USARTx_RX_PIN                    GPIO_PIN_9
#define USARTx_RX_GPIO_PORT              GPIOD
#define USARTx_RX_AF                     GPIO_AF7_USART3

/* Definition for USARTx's NVIC */
#define USARTx_IRQn                      USART3_IRQn
#define USARTx_IRQHandler                USART3_IRQHandler

void UART_init();
int UART_read(uint8_t* data, int max_len);
int UART_write(uint8_t* data, int size);
