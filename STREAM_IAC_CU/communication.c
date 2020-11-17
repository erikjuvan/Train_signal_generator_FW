/// @file communication.c
/// <summary>
/// USB and UART communication library using lower level USB and UART driver functions.
/// </summary>
///
/// Supervision: /
///
/// Company: Sensum d.o.o.
///
/// @authors Erik Juvan
///
/// @version /
/////-----------------------------------------------------------
// Company: Sensum d.o.o.

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

//---------------------------------------------------------------------
/// <summary> See uart.c for documentation. </summary>
//---------------------------------------------------------------------
void UART_RX_Complete_Callback(const uint8_t* data, int size)
{
    rx_buffer_size = size;
    memcpy(rx_buffer, data, size);
    rx_buffer[size] = 0;
    EXTI->SWIER     = EXTI_SWIER_SWIER0; // This triggers EXTI interrupt
}

//---------------------------------------------------------------------
/// <summary> External interrupt on line 0 interrupt handler.
/// This handler is called via software IRQ when data from UART
/// is recevied in its entirety. </summary>
//---------------------------------------------------------------------
void EXTI0_IRQHandler()
{
    EXTI->PR = EXTI_PR_PR0; // Clear pending bit

    // Call actual implementation callback function (in main.c) which is project specific.
    COM_UART_RX_Complete_Callback(rx_buffer, rx_buffer_size);
}

//---------------------------------------------------------------------
/// <summary> Weak callback function that communication driver calls after
/// receving data from UART. UART COM Read is implemented via interrupt
/// to be as quick as possible to react to incoming PLC data.
/// To be implemented by the user! </summary>
///
/// <param name="buf"> Pointer to a buffer that is holding received data. </param>
/// <param name="max_size"> Number of bytes in buffer. </param>
//---------------------------------------------------------------------
__weak void COM_UART_RX_Complete_Callback(uint8_t* buf, int size)
{
    UNUSED(buf);
    UNUSED(size);
}

//---------------------------------------------------------------------
/// <summary> Write data to UART. </summary>
///
/// <param name="buffer"> Pointer to a buffer from which to write data. </param>
/// <param name="size"> Number of bytes to write. </param>
///
/// <returns> Number of written bytes </returns>
//---------------------------------------------------------------------
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

//---------------------------------------------------------------------
/// <summary> Read data from USB. </summary>
///
/// <param name="buffer"> Pointer to a buffer to read data into. </param>
/// <param name="max_size"> Maximum number of bytes to read into buffer. </param>
///
/// <returns> Number of read bytes </returns>
//---------------------------------------------------------------------
int USBRead(uint8_t* buffer, int max_size)
{
    static uint32_t last_read_tick = 0;
    static int      total_read     = 0;
    int             read           = 0;
    // Read while there is data in buffer
    /* NOTE: Added delayed return since there was a problem with USB driver which reads data in 64 byte chunks.
       This will erronously parse data not received in its entirety (over 64 bytes)).
       I have measured, and it takes approx 200 us between receptions of 64 bytes packets. */
    if ((read = VCP_read(&buffer[total_read], max_size - total_read)) > 0) {
        last_read_tick = HAL_GetTick();
        total_read += read;
    } else if (total_read > 0 && ((HAL_GetTick() - last_read_tick) > 30)) { // at least 30 ms!!! JUST SPENT 2 hours!!! chasing a bug which turned out to be that terminal takes even longer than 3ms to send data, so increased it to 30 ms, hopes this never happenes again! Use a different terminal!!!
        buffer[total_read] = 0;
        int tmp            = total_read;
        total_read         = 0;
        return tmp;
    }

    return 0;
}

//---------------------------------------------------------------------
/// <summary> Write data to USB. </summary>
///
/// <param name="buffer"> Pointer to a buffer from which to write data. </param>
/// <param name="size"> Number of bytes to write. </param>
///
/// <returns> Number of written bytes </returns>
//---------------------------------------------------------------------
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