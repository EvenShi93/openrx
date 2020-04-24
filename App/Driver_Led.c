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

#include "Driver_Led.h"
#include "Driver_Flash.h"

void Led_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  RED_LED_OFF;
  GREEN_LED_OFF;
}

void Bind_Led_Status(void)
{
  static int8_t ledcnt = 0;
  GREEN_LED_OFF;
  if (ledcnt == -4)
  {
    ledcnt = RF_Para.Flash_Para.Output_Mode * 2 + 2;
  }
  else
  {
    if (ledcnt > 0)
    {
      if (ledcnt % 2 == 0)
        RED_LED_ON;
      else
        RED_LED_OFF;
    }
    ledcnt--;
  }
}
