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

#include "Driver_Delay.h"

void Delay_Ms(uint16_t ms)
{
  uint32_t temp;
  SysTick->LOAD = (uint32_t)ms * 6000;
  SysTick->VAL = 0x00;
  SysTick->CTRL = 0x01;
  do
  {
    temp = SysTick->CTRL;
  } while (temp & 0x01 && !(temp & (1 << 16)));
  SysTick->CTRL = 0x00;
  SysTick->VAL = 0x00;
}

void Delay_Us(uint16_t us)
{
  uint32_t temp;
  SysTick->LOAD = (uint32_t)us * 6;
  SysTick->VAL = 0x00;
  SysTick->CTRL = 0x01;
  do
  {
    temp = SysTick->CTRL;
  } while (temp & 0x01 && !(temp & (1 << 16)));
  SysTick->CTRL = 0x00;
  SysTick->VAL = 0x00;
}
