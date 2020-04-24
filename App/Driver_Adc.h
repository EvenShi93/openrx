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

#ifndef __DRIVER_ADC_H__
#define __DRIVER_ADC_H__

#include "stm32f0xx.h"

typedef enum
{
  VOLTAGE_INT = 0,
  ADC_NUM,
} Adc_Congig;

extern __IO uint16_t Adc_Sample[ADC_NUM];
extern uint16_t Vol_Int;

void ADC1_Init(void);
void ReadVol_Func(void);

#endif
