#pragma once

// STM HAL Library
#include <stm32f4xx_hal.h>

#define WritePins(port, on, off) port->BSRR = ((uint32_t)on) | (((uint32_t)off) << 16U)
#define SetPins(port, x) port->BSRR = (uint32_t)x
#define ResetPins(port, x) port->BSRR = ((uint32_t)x) << 16U

#define MAX_STATES 50

#define PORT				GPIOD
#define PORT_CLK_ENABLE()	__GPIOD_CLK_ENABLE()	
#define PORT_CLK_DISABLE()	__GPIOD_CLK_DISABLE()	
#define ALL_PINS			GPIO_PIN_All

#define NUM_OF_CHANNELS	16