#include "communication.h"
#include "uart.h"
#include <string.h>

extern uint8_t UART_Address;

static uint8_t rx_buffer[UART_BUFFER_SIZE];
static uint16_t rx_buffer_size;

void UART_RX_Complete_Callback(uint8_t* data, int size) {
	rx_buffer_size = size > sizeof(rx_buffer) ? sizeof(rx_buffer) : size;
	memcpy(rx_buffer, data, rx_buffer_size);	
}

int Com_Read(uint8_t* data, int max_len) {
	
	if (rx_buffer_size <= 0) return 0;
	
	int byte_cnt = 0;
	for (int i = 0; i < rx_buffer_size; ++i) {
		uint8_t tmp_data = rx_buffer[i];
		
		if ((tmp_data & 0x30) == 0x30) { // Data
			if (byte_cnt % 2 == 0)
				data[byte_cnt / 2] = (tmp_data & 0x0F) << 4;
			else 
				data[byte_cnt / 2] |= tmp_data & 0x0F;
			byte_cnt++;
			if ((byte_cnt / 2) >= max_len) { // input data buffer full
				break;
			}
		} else if (tmp_data == 0x1B) { // Escape
			// Do something
		}
	}
	
	rx_buffer_size = 0;
	
	if (byte_cnt % 2)
		return -1;
	else	
		return byte_cnt / 2;
}

int Com_Read_ASCII(uint8_t* data, int max_len) {
	if (rx_buffer_size <= 0) return 0;
	
	int len = rx_buffer_size > max_len ? max_len : rx_buffer_size;
	memcpy(data, rx_buffer, len);
	
	rx_buffer_size = 0;
	
	return len;
}
	
int Com_Write(uint8_t* data, int size) {
	uint8_t buf[UART_BUFFER_SIZE];
	int packet_size = 1 + size * 2; // 1 - address byte, *2 - each byte is split into 2 send bytes
	
	if (packet_size > UART_BUFFER_SIZE)
		return 0;
	
	buf[0] = 0x80 | UART_Address;	// add origin address byte
	for (int i = 0; i < size; ++i) {
		uint8_t tmp_data = data[i];
		buf[1 + i*2] = 0x30 | (tmp_data >> 4);
		buf[2 + i*2] = 0x30 | (tmp_data & 0x0F);
	}
	
	return UART_Write(buf, packet_size);
}

int Com_Write_ASCII(uint8_t* data, int size) {
	uint8_t buf[UART_BUFFER_SIZE];
	int packet_size = 1 + size; // 1 - address byte
	
	if (packet_size > UART_BUFFER_SIZE)
		return 0;
	
	buf[0] = 0x80 | UART_Address;	// add origin address byte
	memcpy(&buf[1], data, size);
	
	return UART_Write(buf, packet_size);
}