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

#ifndef __DRIVER_FLASH_H__
#define __DRIVER_FLASH_H__

#include "stm32f0xx.h"

#define UIDADR 0x1FFFF7AC

#define FLASH_PAGE_SIZE ((uint32_t)0x00000400)
#define Rx_Data_Addr ((uint32_t)0x08007C00)

typedef enum
{
  PWM_Mode = 0,
  SBUS_Mode,
  Output_Mode_Num,
} Output_Mode;

typedef struct
{
  uint32_t Para_Check;
  uint8_t Rf_Tx_Id[4];
  uint8_t Rf_Rx_Id[4];
  uint8_t Freq_Tune;
  uint8_t Device_Num;
  uint8_t Chan_Skip;
  uint8_t Output_Mode;
  uint8_t Hopping_Map[48];
  uint32_t FailSafe_Check;
  uint16_t FailSafe[16];
} para_struct_t;

typedef union {
  uint32_t Flash_Data[25];
  para_struct_t Flash_Para;
} para_union_t;

extern para_union_t RF_Para;

void Flash_Erase(uint32_t WriteAddr, uint8_t NbrOfPage);
void Flash_Program_Word(uint32_t WriteAddr, uint32_t Data);
void Flash_Program_HalfWord(uint32_t WriteAddr, uint16_t Data);
void Flash_Program_Allow(void);
void Flash_Program_Forbid(void);
void Flash_Data_Save(void);
void Rx_Failsafe(void);

#endif
