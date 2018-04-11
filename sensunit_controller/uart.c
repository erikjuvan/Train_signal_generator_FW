#include <string.h>

#include "uart.h"

static UART_HandleTypeDef UartHandle;

static union {
	uint32_t frame;
	struct {
		uint16_t deviceID;
		uint16_t start;		
	};
} UartFrame;

static struct {
	uint8_t data[UART_BUFFER_SIZE];
	int		i;
} uart_rx_buffer;

static struct {
	uint8_t data[UART_BUFFER_SIZE];
	int		i, size;
} uart_tx_buffer;

// IRQ
/////////////////////////////
__attribute__((optimize("O1"))) void USARTx_IRQHandler() {
	uint32_t isrflags = USARTx->ISR;
	uint32_t cr1its = USARTx->CR1;

	if ((isrflags & USART_ISR_RXNE) && (cr1its & USART_CR1_RXNEIE)) {
		if (uart_rx_buffer.i < sizeof(uart_rx_buffer.data)) {
			uart_rx_buffer.data[uart_rx_buffer.i++] = USARTx->RDR;
		}					
	}

	if ((isrflags & USART_ISR_TXE) && (cr1its & USART_CR1_TXEIE)) {
		USARTx->TDR = uart_tx_buffer.data[uart_tx_buffer.i++];
		if (uart_tx_buffer.i >= uart_tx_buffer.size) 
			USARTx->CR1 &= ~USART_CR1_TXEIE;
	}

}
/////////////////////////////

void HAL_UART_MspInit(UART_HandleTypeDef *huart) {  
	
	GPIO_InitTypeDef  GPIO_InitStruct;
  	
	/* Enable GPIO TX/RX clock */
	USARTx_TX_GPIO_CLK_ENABLE();
	USARTx_RX_GPIO_CLK_ENABLE();
	/* Enable USART clock */
	USARTx_CLK_ENABLE(); 

	/* UART TX GPIO pin configuration  */
	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_NOPULL;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FAST;
	GPIO_InitStruct.Alternate = USARTx_TX_AF;
  
	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);
    
	/* UART RX GPIO pin configuration  */
	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USARTx_RX_AF;
    
	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
    
	/* NVIC for USARTx */
	HAL_NVIC_SetPriority(USARTx_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);
}

void UART_init() {
	UartHandle.Instance        = USARTx;

	UartHandle.Init.BaudRate   = 115200;
	UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
	UartHandle.Init.StopBits   = UART_STOPBITS_1;
	UartHandle.Init.Parity     = UART_PARITY_NONE;
	UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
	UartHandle.Init.Mode       = UART_MODE_TX_RX;
	UartHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	
	HAL_UART_Init(&UartHandle);	
	
	UartFrame.start = UART_FRAME_START;
	UartFrame.deviceID = UART_DEVICE_ID;
	
	/*	UART multiprocessor mode with addressing - is problematic in our case beacuse MSB of 8 bit byte is used to indicate an address byte.
		We will be transfering binary data which often his the MSB high.
	HAL_MultiProcessor_Init(&UartHandle, 0x01, UART_WAKEUPMETHOD_ADDRESSMARK);
	HAL_MultiProcessor_EnableMuteMode(&UartHandle);
	HAL_MultiProcessor_EnterMuteMode(&UartHandle);
	*/
	
	USARTx->CR1 |= USART_CR1_RXNEIE;
}

static int UART_read_raw(uint8_t* data, int max_len) {
	USARTx->CR1 &= ~USART_CR1_RXNEIE;
	int len = uart_rx_buffer.i > max_len ? max_len : uart_rx_buffer.i;
	
	memcpy(data, uart_rx_buffer.data, len);
	uart_rx_buffer.i = 0;
	USARTx->CR1 |= USART_CR1_RXNEIE;
	return len;
}

int UART_read(uint8_t* data, int max_len) {	
	int read = 0, tmp = 0;
	
	tmp = UART_read_raw(data, sizeof(UartFrame));
	
	if (tmp == sizeof(UartFrame)) {	// There is data in the buffer
		if (UartFrame.frame != *((uint32_t*)data)) {	// It is meant for me			
			return 0;
		}
	}
	
	do {	// Do while loop with delay, so that entire message is read before being parsed		
		tmp = UART_read_raw(&data[read], max_len - read);
		read += tmp;
		HAL_Delay(1);
	} while (tmp);
	
	return read;			
}

int UART_write(uint8_t* data, int size) {
	int ret = 0;
	
	if (size > 0 && !(USARTx->CR1 & USART_CR1_TXEIE)) {
		uart_tx_buffer.size = size > sizeof(uart_tx_buffer.data) ? sizeof(uart_tx_buffer.data) : size;
		uart_tx_buffer.i = 0;
		memcpy(uart_tx_buffer.data, data, uart_tx_buffer.size);
		USARTx->CR1 |= USART_CR1_TXEIE;
		ret = 1;
	}
	
	return ret;
}
