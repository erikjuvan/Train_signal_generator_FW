#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"
#include <string.h>

extern uint8_t       UART_Address;
extern const uint8_t CharacterMatch;

int VCP_read(void* pBuffer, int size);
int VCP_write(const void* pBuffer, int size);

static uint8_t rx_buffer[UART_BUFFER_SIZE];
static int     rx_buffer_size = 0;

// UART driver Callback
///////////////////////
void UART_RX_Complete_Callback(const uint8_t* data, int size)
{
    rx_buffer_size = size;
    memcpy(rx_buffer, data, size);
    rx_buffer[size] = 0;
    EXTI->SWIER     = EXTI_SWIER_SWIER0; // This triggers EXTI interrupt
}
///////////////////////

// IRQ
//////
static void UART_RX_Process();
void        EXTI0_IRQHandler(void)
{
    EXTI->PR = EXTI_PR_PR0; // Clear pending bit
    UART_RX_Process();
}
//////

static void UART_RX_Process()
{
    COM_UART_RX_Complete_Callback(rx_buffer, rx_buffer_size);
}

__weak void COM_UART_RX_Complete_Callback(uint8_t* buf, int size)
{
}

int UARTWrite(const uint8_t* buffer, int size)
{
    uint8_t buf[UART_BUFFER_SIZE];
    int     len = 0;

    if (size <= 0)
        return 0;

    int packet_size = 1 + size + 1; // 1 - address byte, + size - payload, + 1 - terminating character

    if (packet_size > UART_BUFFER_SIZE)
        return 0;

    buf[0] = 0x80 | UART_Address; // add origin address byte
    memcpy(&buf[1], buffer, size);
    buf[packet_size - 1] = CharacterMatch; // add terminating character
    len                  = UART_Write(buf, packet_size);

    return len;
}

int USBRead(uint8_t* buffer, int max_size)
{
    int len = 0;
    int tmp = 0;
    while ((tmp = VCP_read(&buffer[len], max_size - len)) > 0) {
        len += tmp;
    }
    buffer[len] = 0;

    return len;
}

int USBWrite(const uint8_t* buffer, int size)
{
    uint8_t buf[UART_BUFFER_SIZE];
    int     len = 0;

    if (size <= 0 || size > (UART_BUFFER_SIZE - 1)) // -1 for newline at the end
        return 0;

    memcpy(buf, buffer, size);
    buf[size++] = CharacterMatch; // add terminating character
    len         = VCP_write(buf, size);

    return len; // len will be size + 1 (because of terminating character)
}