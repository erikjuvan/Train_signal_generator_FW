#pragma once

#include <stdint.h>

int Com_Read(uint8_t* data, int max_len);
int Com_Write(uint8_t* data, int size);

int Com_Read_ASCII(uint8_t* data, int max_len);
int Com_Write_ASCII(uint8_t* data, int size);