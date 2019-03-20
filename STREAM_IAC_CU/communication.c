#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"
#include <string.h>

extern uint8_t       UART_Address;
extern const uint8_t CharacterMatch;
extern char          g_VCPInitialized;

int VCP_read(void* pBuffer, int size);
int VCP_write(const void* pBuffer, int size);

CommunicationInterface g_communication_interface = UART;
CommunicationMode      g_communication_mode      = ASCII;

static uint8_t rx_buffer[UART_BUFFER_SIZE];
static int     rx_buffer_size = 0;

// UART driver Callback
///////////////////////
void UART_RX_Complete_Callback(const uint8_t* data, int size)
{
    if (g_communication_interface == UART) {
        rx_buffer_size = size;
        memcpy(rx_buffer, data, size);
        rx_buffer[size] = 0;
        EXTI->SWIER     = EXTI_SWIER_SWIER0; // This triggers EXTI interrupt
    }
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
    // If com mode is binary then extract data
    if (g_communication_mode == BINARY) {
        for (int i = 0; i < rx_buffer_size; ++i) {
            uint8_t rx_byte = rx_buffer[i];
            if ((rx_byte & 0x30) == 0x30) { // Data
                if (i % 2 == 0)
                    rx_buffer[i / 2] = (rx_byte & 0x0F) << 4;
                else
                    rx_buffer[i / 2] |= (rx_byte & 0x0F);
            } else if (rx_byte == 0x1B) { // Escape
                g_communication_mode = ASCII;
                return;
            }
        }
    }

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

    if (g_communication_mode == ASCII) {
        int packet_size = 1 + size + 1; // 1 - address byte, + size - payload, + 1 - terminating character

        if (packet_size > UART_BUFFER_SIZE)
            return 0;

        buf[0] = 0x80 | UART_Address; // add origin address byte
        memcpy(&buf[1], buffer, size);
        buf[packet_size - 1] = CharacterMatch; // add terminating character
        len                  = UART_Write(buf, packet_size);
    } else if (g_communication_mode == BINARY) {
        int packet_size = 1 + size * 2 + 1; // 1 - address byte, size*2 - payload (each byte is split into 2 send bytes), + 1 - terminating character

        if (packet_size > UART_BUFFER_SIZE)
            return 0;

        buf[0] = 0x80 | UART_Address; // add origin address byte
        for (int i = 0; i < size; ++i) {
            uint8_t tmp_data = ((uint8_t*)buffer)[i];
            buf[1 + i * 2]   = 0x30 | (tmp_data >> 4);
            buf[2 + i * 2]   = 0x30 | (tmp_data & 0x0F);
        }

        buf[packet_size - 1] = CharacterMatch; // add terminating character
        len                  = UART_Write(buf, packet_size);
    }

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
    len         = VCP_write(buffer, size);

    return len; // len will be size + 1 (because of terminating character)
}