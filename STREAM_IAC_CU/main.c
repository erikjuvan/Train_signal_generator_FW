/*
DMA2 is chosen because only only DMA2 streams are able to perform memory-to-memory transfers
TIM1 is the trigger for DMA2
TIM1 is in slave mode and is triggerd by TIM2. Reason for using TIM2 is that TIM1 is 16bit and TIM2 is 32bit
TIM2 is where we change all the timing settings (ARR, PSC, CCR1), TIM1 stays at initial values.
PSC		- timing resolution
CCR1	- next DMA trigger g_time
ARR		- sequence period
*/

#include "usbd_cdc_if.h"
#include <usbd_cdc.h>
#include <usbd_core.h>
#include <usbd_desc.h>

#include "communication.h"
#include "main.h"
#include "parse.h"
#include "uart.h"

#define TIMx	TIM5
#define TIMx_IRQHandler TIM5_IRQHandler
#define __TIMx_CLK_ENABLE __TIM5_CLK_ENABLE
#define TIMx_IRQn	TIM5_IRQn

#define DMA_Stream1 DMA1_Stream1
#define DMA_Stream2 DMA1_Stream3

USBD_HandleTypeDef       USBD_Device;
void                     SysTick_Handler(void);
void                     OTG_FS_IRQHandler(void);
extern PCD_HandleTypeDef hpcd;

int         VCP_read(void* pBuffer, int size);
int         VCP_write(const void* pBuffer, int size);
extern char g_VCPInitialized;

const uint32_t GPIOPinArray[] = {
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7,
    GPIO_PIN_8,
    GPIO_PIN_9,
    GPIO_PIN_10,
    GPIO_PIN_11,
    GPIO_PIN_12,
    GPIO_PIN_13,
    GPIO_PIN_14,
    GPIO_PIN_15};

// 0 - active high, 1 - active low
const int IsGPIOReversePin[] = {
    0, // GPIO_PIN_0
    0, // GPIO_PIN_1
    0, // GPIO_PIN_2
    0, // GPIO_PIN_3
    0, // GPIO_PIN_4
    0, // GPIO_PIN_5
    0, // GPIO_PIN_6
    0, // GPIO_PIN_7
    0, // GPIO_PIN_8
    0, // GPIO_PIN_9
    0, // GPIO_PIN_10
    0, // GPIO_PIN_11
    0, // GPIO_PIN_12
    0, // GPIO_PIN_13
    0, // GPIO_PIN_14
    0  // GPIO_PIN_15
};

int g_timer_period_us = 0;

char g_new_settings_received = 0;

static uint32_t g_pins[MAX_STATES] = {0};
static uint32_t g_time[MAX_STATES] = {0};

uint32_t g_pins_shadow[MAX_STATES] = {0};
uint32_t g_time_shadow[MAX_STATES] = {0};
uint32_t g_num_of_entries          = 0;

int idx = 0;

static void Stop();
static void Start();

static char stop_request = 0, stopping_sequence_in_progress = 0;

// printf functionality
/////////////////////////////////////
#include <sys/stat.h>
int _fstat(int fd, struct stat* pStat)
{
    pStat->st_mode = S_IFCHR;
    return 0;
}
int _close(int a) { return -1; }
int _write(int fd, char* pBuffer, int size) { return VCP_write(pBuffer, size); }
int _isatty(int fd) { return 1; }
int _lseek(int a, int b, int c) { return -1; }
int _read(int fd, char* pBuffer, int size)
{
    for (;;) {
        int done = VCP_read(pBuffer, size);
        if (done)
            return done;
    }
}
/////////////////////////////////////

// IRQs
/////////////////////////////////////
void SysTick_Handler(void)
{
    HAL_IncTick();
    HAL_SYSTICK_IRQHandler();
}
void OTG_FS_IRQHandler(void)
{
    HAL_PCD_IRQHandler(&hpcd);
}

void TIMx_IRQHandler()
{
    if (TIMx->SR & TIM_SR_CC4IF) {
        TIMx->SR = ~TIM_SR_CC4IF;
        HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_4);
        GPIOE->BSRR = g_pins[idx];
        TIMx->CCR4  = g_time[idx];
        idx++;
    }

    // If update interrupt
    if (TIMx->SR & TIM_SR_UIF) {
        // Clear Update interrupt pending flag (note: no need for SR &= ~TIM...)
        TIMx->SR = ~TIM_SR_UIF;
        idx      = 0;
        if (stopping_sequence_in_progress) {
            // Stopping sequence ended. It is now safe to stop everything.
            Stop();
            // Leave one pulse mode
            TIMx->CR1 &= ~TIM_CR1_OPM;
            // Clear all stopping flags
            stopping_sequence_in_progress = 0;
            stop_request                  = 0;
            HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
        } else if (stop_request) {
            // On stop request enter one pulse mode
            TIMx->CR1 |= TIM_CR1_OPM;
            // Flag to signal that the final stopping sequence is active (ongoing)
            stopping_sequence_in_progress = 1;
        }
    }
}
/////////////////////////////////////

static void SystemClock_Config(void)
{

    RCC_ClkInitTypeDef       RCC_ClkInitStruct;
    RCC_OscInitTypeDef       RCC_OscInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

    /* Enable HSE Oscillator and activate PLL with HSE as source */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
    RCC_OscInitStruct.HSIState       = RCC_HSI_OFF;
    RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM       = 8;
    RCC_OscInitStruct.PLL.PLLN       = 336;
    RCC_OscInitStruct.PLL.PLLP       = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ       = 7;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        asm("bkpt 255");
    }

    /* Select PLLQ output as USB clock source */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
    PeriphClkInitStruct.Clk48ClockSelection  = RCC_CLK48SOURCE_PLL;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
        asm("bkpt 255");
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	clocks dividers */
    RCC_ClkInitStruct.ClockType      = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK) {
        asm("bkpt 255");
    }
}

static void SetInitialGPIOState()
{
    // Set intial state
    for (int i = 0; i < NUM_OF_CHANNELS; ++i) {
        HAL_GPIO_WritePin(PORT, GPIOPinArray[i], (GPIO_PinState)IsGPIOReversePin[i]);
    }
}

static void GPIO_Configure()
{
    PORT_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull  = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStructure.Pin   = ALL_PINS;

    HAL_GPIO_Init(PORT, &GPIO_InitStructure);

    SetInitialGPIOState();

    __GPIOB_CLK_ENABLE();
    GPIO_InitStructure.Pin = GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4;

    HAL_GPIO_Init(GPIOB, &GPIO_InitStructure);
}

static void EXTI_Configure()
{
    EXTI->IMR |= EXTI_IMR_IM0;
    HAL_NVIC_SetPriority(EXTI0_IRQn, 3, 0);
    HAL_NVIC_EnableIRQ(EXTI0_IRQn);
}

// DMA
/////////////////////////////////////
static void DMA_Configure()
{
    __DMA1_CLK_ENABLE();
    DMA_Stream1->NDTR = g_num_of_entries;
    DMA_Stream1->M0AR = (uint32_t)g_pins;
    DMA_Stream1->PAR  = (uint32_t)&PORT->BSRR;
    DMA_Stream1->CR   = DMA_CHANNEL_6 | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE | DMA_PRIORITY_VERY_HIGH | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
                       DMA_MINC_ENABLE | DMA_CIRCULAR | DMA_MEMORY_TO_PERIPH;

    DMA_Stream2->NDTR = g_num_of_entries;
    DMA_Stream2->M0AR = (uint32_t)g_time;
    DMA_Stream2->PAR  = (uint32_t)&TIMx->CCR4;
    DMA_Stream2->CR   = DMA_CHANNEL_6 | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE | DMA_PRIORITY_HIGH | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
                       DMA_MINC_ENABLE | DMA_CIRCULAR | DMA_MEMORY_TO_PERIPH;
}

static void DMA_Start()
{
    // First CLEAR LISR and HISR event flags
    DMA1->HIFCR = ~0x0; // clear all

    DMA_Stream1->CR |= DMA_SxCR_EN;
    DMA_Stream2->CR |= DMA_SxCR_EN;

    while (!(DMA_Stream1->CR & DMA_SxCR_EN) || !(DMA_Stream2->CR & DMA_SxCR_EN))
        ; // wait for CE to be read as 1
}
static void DMA_Stop()
{
    DMA_Stream1->CR &= ~DMA_SxCR_EN;
    DMA_Stream2->CR &= ~DMA_SxCR_EN;

    while (DMA_Stream1->CR & DMA_SxCR_EN || DMA_Stream2->CR & DMA_SxCR_EN)
        ; // wait for CE to be read as 0
}
static void DMA_Update(uint32_t n_entries)
{
    // Only update when DMA is disabled
    if (!(DMA_Stream1->CR & DMA_SxCR_EN) && !(DMA_Stream2->CR & DMA_SxCR_EN)) {
        DMA_Stream1->NDTR = n_entries;
        DMA_Stream2->NDTR = n_entries;
    }
}
/////////////////////////////////////

// TIM
/////////////////////////////////////
static void TIM_Configure()
{
    // TIM2 has to be configured before TIM1 otherwise TIM2 causes TIM1 IRQ when executing TIM2->EGR = TIM_EGR_UG;
    __TIMx_CLK_ENABLE();
    TIMx->PSC = (uint32_t)(SystemCoreClock / 2) / 1e6 / 4 - 1; // Prescaler value that comes to one tick being 250 ns
    //TIMx->CR2 |= TIM_CR2_MMS_0 | TIM_CR2_MMS_1;
    //TIMx->CR2 |= TIM_CR2_CCDS;
    TIMx->EGR = TIM_EGR_UG;     // Generate update event (this also loads the prescaler)
    TIMx->SR  = 0;              // Clear update event in the status register that we triggered in the line above
    TIMx->DIER |= TIM_DIER_UIE | TIM_DIER_CC4DE | TIM_DIER_CC4IE | TIM_DIER_TDE; // Enable update interrupt
    // Thougt I needed this, turns out I don't, I needed it because I updated ARR somewhere async while timer was running, and this prevented ARR from updating on the spot.
    // But now with improvements to the code, parser no longer directly configures peripherals.
    //TIM2->CR1 |= TIM_CR1_ARPE; // Auto reload register is preloaded (ref. page 711)

    /*
    __TIM1_CLK_ENABLE();
    TIM1->DIER |= TIM_DIER_TDE;
    TIM1->SMCR |= TIM_SMCR_TS_0;
    TIM1->SMCR |= TIM_SMCR_SMS_2;
*/
    // Enable TIM2 interrupts
    HAL_NVIC_SetPriority(TIMx_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIMx_IRQn);


    /* Use for testing (NOT TESTED)
	// IRQ enable TIM1
	TIM1->DIER |= TIM_DIER_TIE;
	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
        
    // IRQ handler
	void TIM1_TRG_COM_TIM11_IRQHandler() {	
		TIM1->SR &= ~TIM_SR_TIF;
		asm("nop");
	}
	*/
}

static void TIM_Start()
{
    TIMx->CR1 |= TIM_CR1_CEN;
}
static void TIM_Stop()
{
    TIMx->CR1 &= ~TIM_CR1_CEN;
}
static void TIM_Update_ARR(uint32_t arr)
{
    TIMx->ARR = arr;
}
/////////////////////////////////////

// TODO: Think about maybe calling this and StopRequest from main where we check flags
void StartRequest()
{
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    // Wait while stop request is in progress
    while (stop_request) // NOTE: while (TIMx->CR1 & TIM_CR1_OPM) for some reason corrupts the stopping pulse train (maybe reading it too fast causes trouble)
        ;

    if (g_new_settings_received) {
        g_new_settings_received = 0;

        // Copy values from shadow registers
        for (int i = 0; i < g_num_of_entries; ++i) {
            g_pins[i] = g_pins_shadow[i];
            g_time[i] = g_time_shadow[i];
        }

        DMA_Update(g_num_of_entries);

        // Increase by magic number 4 which is tied to the magic number in the PSC to get exactly 1us resolution
        TIM_Update_ARR(g_timer_period_us * 4);

        // Update CCR1 register with the last entry in the time array which is the time at which the first GPIO change should happen
        // NOTE: First entry in the settings can't be 0 (TODO: look into it if there is a way to allow starting with 0)
        TIMx->CCR4 = g_time[g_num_of_entries - 1];
    }

    Start();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
}

static void Start()
{
    DMA_Start();
    TIM_Start();
}

void StopRequest()
{
    // If timer is not running just return since it is already stopped
    if (!(TIMx->CR1 & TIM_CR1_CEN))
        return;

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
    stop_request = 1;
}

static void Stop()
{
    TIM_Stop();
    DMA_Stop();

    SetInitialGPIOState();
}

static void USB_Init()
{
    USBD_Init(&USBD_Device, &VCP_Desc, 0);

    USBD_RegisterClass(&USBD_Device, &USBD_CDC);
    USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_STREAM_IAC_CU_fops);
    USBD_Start(&USBD_Device);
}

static void USB_Deinit()
{
    USBD_Stop(&USBD_Device);
    USBD_DeInit(&USBD_Device);
}

static void Init()
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Configure();
    DMA_Configure();
    TIM_Configure();
    EXTI_Configure();

    UART_Init();

    USB_Init();
}

void COM_UART_RX_Complete_Callback(uint8_t* buf, int size)
{
    Parse((char*)buf, UARTWrite);
}

int SimpleUARTWrite(const uint8_t* buf, int size)
{
    for (int i = 0; i < size; ++i) {
        while (!(USARTx->ISR & USART_ISR_TXE))
            ;
        USARTx->TDR = buf[i];
    }
    while (!(USARTx->ISR & USART_ISR_TXE))
            ;
    USARTx->TDR = '\n';
    
    return size+1;
}

void SimpleUARTRead()
{
    static char uart_rx_buf[UART_BUFFER_SIZE] = {0};
    static int  uart_rx_i                     = 0;
    
    // UART Read       
    if (USARTx->ISR & USART_ISR_RXNE) {
        uart_rx_buf[uart_rx_i] = USARTx->RDR;
        if (uart_rx_buf[uart_rx_i] == 0xFF) {
            uart_rx_buf[uart_rx_i] = 0;
            Parse(uart_rx_buf, SimpleUARTWrite);
            uart_rx_i = 0;
        } else {
            uart_rx_i++;
        }            
    }
}

int main()
{
    uint8_t rxBuf[UART_BUFFER_SIZE] = {0};
    int     usb_read                = 0;

    Init();

    while (1) {

        if (g_VCPInitialized) { // Make sure USB is initialized (calling, VCP_write can halt the system if the data structure hasn't been malloc-ed yet)
            usb_read = USBRead(rxBuf, sizeof(rxBuf));
            if (usb_read > 0) {
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
                Parse((char*)rxBuf, USBWrite);
                HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
                memset(rxBuf, 0, usb_read);
            }
        }

        //SimpleUARTRead();
    }
}
