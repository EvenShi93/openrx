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

#include "Driver_Flash.h"
#include "Driver_Output.h"
#include "Driver_Delay.h"
#include "Driver_Led.h"
#include "Protocol.h"
#include "string.h"

uint8_t FailSafe_Flag;

#define FLASH_PAGES_TO_BE_PROTECTED OB_WRP_Pages0to3

void Flash_Erase(uint32_t addr, uint8_t NbrOfPage)
{
  uint8_t i = 0;
  FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
  for (i = 0; i < NbrOfPage; i++)
  {
    FLASH_ErasePage(addr + (FLASH_PAGE_SIZE * i));
  }
}

void Flash_Program_Word(uint32_t WriteAddr, uint32_t Data)
{
  if (FLASH_ProgramWord(WriteAddr, Data) != FLASH_COMPLETE)
  {
    while (1)
    {
      RED_LED_FLICKER;
      Delay_Ms(200);
    }
  }
}

void Flash_Program_HalfWord(uint32_t WriteAddr, uint16_t Data)
{
  if (FLASH_ProgramHalfWord(WriteAddr, Data) != FLASH_COMPLETE)
  {
    while (1)
    {
      RED_LED_FLICKER;
      Delay_Ms(200);
    }
  }
}

void Flash_Program_Allow(void)
{
  __set_PRIMASK(1);
  Delay_Ms(1);
  FLASH_Unlock();
}

void Flash_Program_Forbid(void)
{
  FLASH_Lock();
  Delay_Ms(1);
  __set_PRIMASK(0);
}

void Flash_Data_Save(void)
{
  Flash_Program_Allow();
  Flash_Erase(Rx_Data_Addr, 1);
  for (uint16_t i = 0; i < 25; i++)
    Flash_Program_Word(Rx_Data_Addr + 4 * i, RF_Para.Flash_Data[i]);
  Flash_Program_Forbid();
}

void Rx_Failsafe(void)
{
  static uint16_t FailSafe_Cnt = 0;

  if (FailSafe_Flag == 0)
  {
    if ((GPIOA->IDR & GPIO_Pin_1) == 0)
    {
      FailSafe_Cnt++;
      if (FailSafe_Cnt > 100)
      {
        for (uint8_t i = 0; i < 8; i++)
        {
          RF_Para.Flash_Para.FailSafe[i] = Channel_Output[i];
        }
        RF_Para.Flash_Para.FailSafe_Check = 0x4B424A4F;
        Flash_Program_Allow();
        for (uint8_t i = 0; i < 10; i++)
        {
          RED_LED_ON;
          GREEN_LED_OFF;
          Delay_Ms(100);
          RED_LED_OFF;
          GREEN_LED_ON;
          Delay_Ms(100);
        }
        Flash_Data_Save();
        FailSafe_Flag = 1;
        A7105_GDO_Init();
        TimeOut_Init();
      }
    }
  }
}
