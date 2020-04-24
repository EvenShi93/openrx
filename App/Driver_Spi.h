/*
 * Copyright (C) MerakRC
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef __DRIVER_SPI_H__
#define __DRIVER_SPI_H__

#include "stm32f0xx.h"

#define RFChip_Enable GPIOA->BRR = GPIO_Pin_3
#define RFChip_Disable GPIOA->BSRR = GPIO_Pin_3

void Rfchip_Spi_Init(void);
void Rf_Spi_Write_Byte(uint8_t dat);
uint8_t Rf_Spi_Read_Byte(void);

#endif
