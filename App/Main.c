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

#include "Driver_Adc.h"
#include "Driver_Button.h"
#include "Driver_Delay.h"
#include "Driver_Flash.h"
#include "Driver_Led.h"
#include "Driver_Output.h"
#include "Driver_Spi.h"
#include "Protocol.h"
#include "A7105.h"

int main(void)
{
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOFEN;
  SysTim_Init();
  Led_Init();
  Button_Init();
  Rfchip_Spi_Init();
  Afhds_Init();
  Afhds_Loop();
}
