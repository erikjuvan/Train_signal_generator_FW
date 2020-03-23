#pragma once

// STM HAL Library
#include <stm32f7xx_hal.h>

#define SWVER "IAC_CU V1.0.0.0"
#define HWVER "N/A"
#define COMPATIBILITYMODE "N/A"

#define WritePins(port, on, off) port->BSRR = ((uint32_t)on) | (((uint32_t)off) << 16U)
#define SetPins(port, x) port->BSRR = (uint32_t)x
#define ResetPins(port, x) port->BSRR = ((uint32_t)x) << 16U

#define MAX_STATES 64

#define PORT GPIOE
#define PORT_CLK_ENABLE() __GPIOE_CLK_ENABLE()
#define PORT_CLK_DISABLE() __GPIOE_CLK_DISABLE()
#define ALL_PINS GPIO_PIN_All

#define NUM_OF_CHANNELS 16
