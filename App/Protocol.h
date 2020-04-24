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

#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include "stm32f0xx.h"

void Afhds_Init(void);
void Afhds_Loop(void);
void Afhds_A7105_Init(void);
void Afhds_Bind(void);
void A7105_SetChan(uint8_t skip, uint8_t num);
void A7105_GDO_Init(void);
void TimeOut_Init(void);
void SysTim_Init(void);

#endif
