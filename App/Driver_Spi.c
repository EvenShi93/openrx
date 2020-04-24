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

#include "Driver_Spi.h"
#include "Driver_Delay.h"

void Rfchip_Spi_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5 | GPIO_Pin_7;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	GPIOA->BRR = GPIO_Pin_5;
	GPIOA->BRR = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
	GPIO_Init(GPIOF, &GPIO_InitStructure);
	GPIOF->BRR = GPIO_Pin_0;
	GPIOF->BRR = GPIO_Pin_1;

	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	RFChip_Disable;
}

void Rf_Spi_Write_Byte(uint8_t dat)
{
	GPIOA->BRR = GPIO_Pin_5;
	GPIOA->MODER = ((GPIOA->MODER & (~(GPIO_MODER_MODER0 << 14))) | (GPIO_Mode_OUT << 14));
	for (uint8_t i = 0; i < 8; i++)
	{
		if ((dat & 0x80) == 0x80)
			GPIOA->BSRR = GPIO_Pin_7;
		else
			GPIOA->BRR = GPIO_Pin_7;
		GPIOA->BSRR = GPIO_Pin_5;
		dat <<= 1;
		GPIOA->BRR = GPIO_Pin_5;
	}
}

uint8_t Rf_Spi_Read_Byte(void)
{
	uint8_t dat;
	GPIOA->BRR = GPIO_Pin_5;
	GPIOA->MODER = ((GPIOA->MODER & (~(GPIO_MODER_MODER0 << 14))) | (GPIO_Mode_IN << 14));
	for (uint8_t i = 0; i < 8; i++)
	{
		dat <<= 1;
		GPIOA->BSRR = GPIO_Pin_5;
		if (GPIOA->IDR & GPIO_Pin_7)
			dat |= 0x01;
		GPIOA->BRR = GPIO_Pin_5;
	}
	return dat;
}
