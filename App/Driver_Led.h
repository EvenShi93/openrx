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

#ifndef __DRIVER_LED_H__
#define __DRIVER_LED_H__

#include "stm32f0xx.h"

#define RED_LED_FLICKER GPIOA->ODR ^= GPIO_Pin_14
#define RED_LED_OFF GPIOA->BRR = GPIO_Pin_14
#define RED_LED_ON GPIOA->BSRR = GPIO_Pin_14

#define GREEN_LED_FLICKER GPIOA->ODR ^= GPIO_Pin_13
#define GREEN_LED_OFF GPIOA->BRR = GPIO_Pin_13
#define GREEN_LED_ON GPIOA->BSRR = GPIO_Pin_13

void Led_Init(void);
void Bind_Led_Status(void);

#endif
