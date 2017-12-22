/*
DMA2 is chosen because only only DMA2 streams are able to perform memory-to-memory transfers
TIM1 is the trigger for DMA2
TIM1 is in slave mode and is triggerd by TIM2. Reason for using TIM2 is that TIM1 is 16bit and TIM2 is 32bit
TIM2 is where we change all the timing settings (ARR, PSC, CCR1), TIM1 stays at initial values.
PSC		- timing resolution
CCR1	- next DMA trigger g_time
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
	0, 	// GPIO_PIN_2
	0, 	// GPIO_PIN_3
	0, 	// GPIO_PIN_4
	0, 	// GPIO_PIN_5
	0,	// GPIO_PIN_6
	0,	// GPIO_PIN_7
	0, 	// GPIO_PIN_8
	0, 	// GPIO_PIN_9
	0, 	// GPIO_PIN_10
	1, 	// GPIO_PIN_11
	1,	// GPIO_PIN_12
	1,	// GPIO_PIN_13
	1, 	// GPIO_PIN_14
	1	// GPIO_PIN_15
};

uint32_t	g_pins[MAX_STATES] = { 0 };
uint32_t	g_time[MAX_STATES] = { 0 };
uint32_t	num_of_entries = 0;


// Debug
/////////////////////////////////////

// Debug GPIO
#define DEBUG_PORT		GPIOF
#define DEBUG_PIN		GPIO_PIN_8
#define DEBUG_CLK()		__GPIOF_CLK_ENABLE()
#define DEBUG_SET()		DEBUG_PORT->BSRR = DEBUG_PIN
#define DEBUG_RESET()	DEBUG_PORT->BSRR = DEBUG_PIN << 16

// printf functionality
/////////////////////////////////////
#include<sys/stat.h>
int _fstat (int fd, struct stat *pStat) {
	pStat->st_mode = S_IFCHR;
	return 0;
}
int _close(int a) { return -1; }
int _write (int fd, char *pBuffer, int size) { return VCP_write(pBuffer, size); }
int _isatty (int fd) { return 1; }
int _lseek(int a, int b, int c) { return -1; }
int _read (int fd, char *pBuffer, int size) {
	for (;;) {
		int done = VCP_read(pBuffer, size);
		if (done) return done;
	}
}
/////////////////////////////////////

// IRQs
/////////////////////////////////////
void SysTick_Handler(void) {
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
}
void OTG_FS_IRQHandler(void) {
	HAL_PCD_IRQHandler(&hpcd);
}
/////////////////////////////////////

static void SystemClock_Config(void) {
	
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;
  
	/* Enable HSE Oscillator and activate PLL with HSE as source */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSIState = RCC_HSI_OFF;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 8;
	RCC_OscInitStruct.PLL.PLLN = 432;  
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		asm("bkpt 255");
	}
  
	/* Activate the OverDrive to reach the 216 Mhz Frequency */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		asm("bkpt 255");
	}
  
	/* Select PLLSAI output as USB clock source */
	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct)  != HAL_OK) {
		asm("bkpt 255");
	}
  
	/* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
	clocks dividers */
	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		asm("bkpt 255");
	}
}


static void SetInitialGPIOState() {
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
	
	// Debug
	DEBUG_CLK();
	GPIO_InitStructure.Pin = DEBUG_PIN;
	HAL_GPIO_Init(DEBUG_PORT, &GPIO_InitStructure);
}


// DMA
/////////////////////////////////////
static void DMA_Configure() {
	__DMA2_CLK_ENABLE();
	DMA2_Stream0->NDTR = num_of_entries;
	DMA2_Stream0->M0AR = (uint32_t) g_pins;
	DMA2_Stream0->PAR = (uint32_t) &PORT->BSRR;
	DMA2_Stream0->CR = DMA_CHANNEL_6 | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE | DMA_PRIORITY_VERY_HIGH | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
		DMA_MINC_ENABLE | DMA_CIRCULAR | DMA_MEMORY_TO_PERIPH;
	
	DMA2_Stream4->NDTR = num_of_entries;
	DMA2_Stream4->M0AR = (uint32_t) g_time;
	DMA2_Stream4->PAR = (uint32_t) &TIM2->CCR1;
	DMA2_Stream4->CR = DMA_CHANNEL_6 | DMA_MBURST_SINGLE | DMA_PBURST_SINGLE | DMA_PRIORITY_HIGH | DMA_SxCR_MSIZE_1 | DMA_SxCR_PSIZE_1 |
		DMA_MINC_ENABLE | DMA_CIRCULAR | DMA_MEMORY_TO_PERIPH;
}

static void DMA_Start() {
	DMA2_Stream0->CR |= DMA_SxCR_EN;
	DMA2_Stream4->CR |= DMA_SxCR_EN;		
	
	while (!(DMA2_Stream0->CR & DMA_SxCR_EN) || !(DMA2_Stream4->CR & DMA_SxCR_EN)) ;	// wait for CE to be read as 1
}

static void DMA_Stop() {
	DMA2_Stream0->CR &= ~DMA_SxCR_EN;
	DMA2_Stream4->CR &= ~DMA_SxCR_EN;		

	while (DMA2_Stream0->CR & DMA_SxCR_EN || DMA2_Stream4->CR & DMA_SxCR_EN) ;	// wait for CE to be read as 0
}

void DMA_Update(uint32_t n_entries) {
	DMA2_Stream0->NDTR = n_entries;
	DMA2_Stream4->NDTR = n_entries;
}
/////////////////////////////////////

// TIM
/////////////////////////////////////
static void TIM_Configure() {
	// TIM2 has to be configured before TIM1 otherwise TIM2 causes TIM1 IRQ when executing TIM2->EGR = TIM_EGR_UG;
	__TIM2_CLK_ENABLE();
	TIM2->PSC = 26;   			// Prescaler value 26, that comes to one tick being 249 ns
	TIM2->ARR = 0xFFFF;   		// Reload timer
	TIM2->CCR1 = 1;		
	TIM2->CR2 |= TIM_CR2_MMS_0 | TIM_CR2_MMS_1;
	TIM2->EGR = TIM_EGR_UG;   	// Reset the counter and generate update event
	TIM2->SR = 0;	// Clear interrupts
	
	__TIM1_CLK_ENABLE();
	TIM1->DIER |= TIM_DIER_TDE;
	TIM1->SMCR |= TIM_SMCR_TS_0;
	TIM1->SMCR |= TIM_SMCR_SMS_2 | TIM_SMCR_SMS_1;   	// I don't completely understand why it has to be exactly so	
		
	/* Use for testing 
	// IRQ enable
	TIM1->DIER |= TIM_DIER_TIE;
	HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
	
	// IRQ handler
	void TIM1_TRG_COM_TIM11_IRQHandler() {	
		TIM1->SR = ~TIM_DIER_TIE;
		asm("nop");
	}	
	*/
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
	// ORDER OF STATEMENTS MATTERS!!!
	TIM2->CCR1 = 1;
	TIM2->EGR = TIM_EGR_UG;		
}
/////////////////////////////////////

void Stop() {
	// ORDER OF FUNCTION CALLS MATTERS!!!
	TIM_Stop();
	DMA_Stop();	
	TIM_Update_REGS();
	
	SetInitialGPIOState();
	HAL_Delay(100);
}

void Start() {
	DMA_Start();
	TIM_Start();
}

static void Init() {
	HAL_Init();
	SystemClock_Config();

	USBD_Init(&USBD_Device, &VCP_Desc, 0);

	USBD_RegisterClass(&USBD_Device, &USBD_CDC);
	USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_sensunit_controller_fops);
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
		
		do {	// Do while loop with delay, so that entire message is read before being parsed
			tmp = VCP_read(&rxBuf[read], sizeof(rxBuf) - read);
			read += tmp;
			HAL_Delay(10);
		} while (tmp);		
		
		if (read > 0) {					
			ParseScript((char*)rxBuf);
			memset(rxBuf, 0, read);
			read = 0;
		}						
	}
}


