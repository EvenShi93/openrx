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

#ifndef __DRIVER_OUTPUT_H__
#define __DRIVER_OUTPUT_H__

#include "stm32f0xx.h"

struct sbus_dat
{
  uint8_t syncByte;
  unsigned int ch0 : 11;
  unsigned int ch1 : 11;
  unsigned int ch2 : 11;
  unsigned int ch3 : 11;
  unsigned int ch4 : 11;
  unsigned int ch5 : 11;
  unsigned int ch6 : 11;
  unsigned int ch7 : 11;
  unsigned int ch8 : 11;
  unsigned int ch9 : 11;
  unsigned int ch10 : 11;
  unsigned int ch11 : 11;
  unsigned int ch12 : 11;
  unsigned int ch13 : 11;
  unsigned int ch14 : 11;
  unsigned int ch15 : 11;
  uint8_t flags;
  uint8_t endByte;
} __attribute__((__packed__));

typedef union {
  uint8_t byte[25];
  struct sbus_dat msg;
} sbus_msg;

extern sbus_msg sbus;
extern volatile uint16_t Channel_Output[16];

void Port_Init(void);
void Port_Output(void);

#endif
