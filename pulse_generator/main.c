/*

DMA2 is chosen because only only DMA2 streams are able to perform memory-to-memory transfers
TIM1 is the trigger for DMA2
TIM1 is in slave mode and is triggerd by TIM2. Reason for using TIM2 is that TIM1 is 16bit and TIM2 is 32bit
TIM2 is where we change all the timing settings (ARR, PSC, CCR1), TIM1 stays at initial values.
PSC		- timing resolution
CCR1	- next DMA trigger time
ARR		- sequence period

*/

#include <usbd_core.h>
#include <usbd_cdc.h>
#include "usbd_cdc_if.h"
#include <usbd_desc.h>

#include "main.h"

USBD_HandleTypeDef USBD_Device;
void SysTick_Handler(void);
void OTG_FS_IRQHandler(void);
extern PCD_HandleTypeDef hpcd;
	
int VCP_read(void *pBuffer, int size);
int VCP_write(const void *pBuffer, int size);
extern char g_VCPInitialized;

extern void ParseScript(char* script);

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
	GPIO_PIN_15
};

// 0 - active high, 1 - active low
const int IsGPIOReversePin[] = {
	0,	// GPIO_PIN_0
	0,	// GPIO_PIN_1
	0,	// GPIO_PIN_2
	0,	// GPIO_PIN_3
	0,	// GPIO_PIN_4
	0,	// GPIO_PIN_5
	0,	// GPIO_PIN_6
	0,	// GPIO_PIN_7
	0,	// GPIO_PIN_8
	0,	// GPIO_PIN_9
	0,	// GPIO_PIN_10
	1,	// GPIO_PIN_11
	1,	// GPIO_PIN_12
	1,	// GPIO_PIN_13
	1,	// GPIO_PIN_14
	1	// GPIO_PIN_15
};

uint32_t	pins[MAX_STATES] = { 0 };
uint32_t	time[MAX_STATES] = { 0 };
uint32_t	num_of_entries = 0;

// IRQ
/////////////////////////////////////
void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}

void OTG_FS_IRQHandler(void) {
	HAL_PCD_IRQHandler(&hpcd);
}

// System clock is configured for the max: 168 MHz
static void SystemClock_Config(void) {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 336;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 7;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);

	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);	
}

void SetInitialGPIOState() {
	// Set intial state
	for(int i = 0 ; i < NUM_OF_CHANNELS ; i++) {
		HAL_GPIO_WritePin(PORT, GPIOPinArray[i], (GPIO_PinState) IsGPIOReversePin[i]);
	}
}

static void GPIO_Configure() {
	PORT_CLK_ENABLE();
	GPIO_InitTypeDef GPIO_InitStructure;
	
	GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStructure.Pull = GPIO_NOPULL;
	GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
	GPIO_InitStructure.Pin = ALL_PINS;		
		
	HAL_GPIO_Init(PORT, &GPIO_InitStructure);
	
	SetInitialGPIOState();
}

static void DMA_Configure() {
	__DMA2_CLK_ENABLE();
	DMA2_Stream0->NDTR = num_of_entries;
	DMA2_Stream0->M0AR = (uint32_t) pins;
	DMA2_Stream0->PAR = (uint32_t) &PORT->BSRR;
	DMA2_Stream0->CR = DMA_CHANNEL_6 | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE | DMA_PRIORITY_VERY_HIGH | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
		DMA_MINC_ENABLE | DMA_CIRCULAR | DMA_MEMORY_TO_PERIPH;
	
	DMA2_Stream4->NDTR = num_of_entries;
	DMA2_Stream4->M0AR = (uint32_t) time;
	DMA2_Stream4->PAR = (uint32_t) &TIM2->CCR1;
	DMA2_Stream4->CR = DMA_CHANNEL_6 | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE | DMA_PRIORITY_HIGH | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
		DMA_MINC_ENABLE | DMA_CIRCULAR | DMA_MEMORY_TO_PERIPH;
}

static void TIM_Configure() {
	__TIM1_CLK_ENABLE();
	TIM1->PSC = 0; 			// Set the Prescaler value
	TIM1->ARR = 0; 			// Reload timer
	TIM1->DIER |= TIM_DIER_TDE;
	TIM1->SMCR |= TIM_SMCR_TS_0;
	TIM1->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1; 	// I don't understand why it has to be exactly so
	
	__TIM2_CLK_ENABLE();
	TIM2->PSC = 20; 			// Set the Prescaler value: 20, that comes to one tick being 249 ns
	TIM2->ARR = 0xFFFF; 		// Reload timer
	TIM2->CCR1 = 0x10;
	TIM2->EGR = TIM_EGR_UG; 	// Reset the counter and generate update event		
	TIM2->CR2 |= TIM_CR2_MMS_0 | TIM_CR2_MMS_1;
}

static void DMA_Start() {
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream4->CR |= DMA_SxCR_EN;		
	
	while (!(DMA2_Stream0->CR & DMA_SxCR_EN) || !(DMA2_Stream4->CR & DMA_SxCR_EN))
		;	// wait for CE to be read as 1
}

static void DMA_Stop() {	
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream4->CR &= ~DMA_SxCR_EN;		

	while (DMA2_Stream0->CR & DMA_SxCR_EN || DMA2_Stream4->CR & DMA_SxCR_EN)
		;	// wait for CE to be read as 0
}

void DMA_Update(uint32_t n_entries) {
	DMA2_Stream0->NDTR = n_entries;
	DMA2_Stream4->NDTR = n_entries;
}

static void TIM_Start() {
	TIM2->CR1 |= TIM_CR1_CEN;
}

static void TIM_Stop() {
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

void TIM_Update_ARR(uint32_t arr) {	
	TIM2->ARR = arr;	
}
void TIM_Update_PSC(uint32_t psc) {	
	TIM2->PSC = psc;
}
void TIM_Update_REGS() {	
	TIM2->EGR = TIM_EGR_UG; 	// Update registers
}

void Stop() {
	//TIM_Stop();	// Stopping timer for some reason inverts the output sometimes (no idea why)
	DMA_Stop();
}

void Start() {
	PORT_CLK_DISABLE(); 	// Hack 2, to prevent triggering for an entire period because of hack 1. When first loading the program the problematic trigger still ocurs.
	HAL_Delay(10);
	DMA_Start();
	TIM_Start();
	
	// Hack 1, needed to prevent +1 offset between dma streams. Why... I do not know!
	DMA_Stop();
	DMA_Start();	
	PORT_CLK_ENABLE(); 	// Hack 2
}

static void Init() {
	HAL_Init();
	SystemClock_Config();

	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_pulse_generator_fops);
	USBD_Start(&USBD_Device);

	while (USBD_Device.pClassData == 0) {
	}
	
	GPIO_Configure();
	DMA_Configure();
	TIM_Configure();
}

int main() {
	int read = 0, tmp = 0;
	uint8_t rxBuf[1024] = { 0 };
	
	Init();	
		
	while (1) {						
		// Do while loop with delay, so that entire message is read before being parsed
		do {
			tmp = VCP_read(&rxBuf[read], sizeof(rxBuf) - read);
			read += tmp;
			HAL_Delay(10);
		} while (tmp)
			;		
		
		if (read > 0) {					
			ParseScript((char*)rxBuf);
			memset(rxBuf, 0, read);
			read = 0;
		}						
	}
}
