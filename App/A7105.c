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

#include "A7105.h"
#include "Driver_Spi.h"

void A7105_Reset(void)
{
  A7105_WriteReg(A7105_REG_MODE, 0x00);
}

void A7105_Strobe(uint8_t strobe)
{
  RFChip_Enable;
  Rf_Spi_Write_Byte(strobe);
  RFChip_Disable;
}

void A7105_WriteReg(uint8_t reg, uint8_t data)
{
  RFChip_Enable;
  Rf_Spi_Write_Byte(reg);
  Rf_Spi_Write_Byte(data);
  RFChip_Disable;
}

uint8_t A7105_ReadReg(uint8_t reg)
{
  uint8_t data;

  RFChip_Enable;
  Rf_Spi_Write_Byte(reg | 0x40);
  data = Rf_Spi_Read_Byte();
  RFChip_Disable;
  return data;
}

void A7105_WriteFIFO(uint8_t *packet, uint8_t length)
{
  RFChip_Enable;
  Rf_Spi_Write_Byte(A7105_REG_FIFO_DATA);
  for (uint8_t i = 0; i < length; i++)
  {
    Rf_Spi_Write_Byte(*packet++);
  }
  RFChip_Disable;
}

void A7105_ReadFIFO(uint8_t *packet, uint8_t length)
{
  RFChip_Enable;
  Rf_Spi_Write_Byte(A7105_REG_FIFO_DATA | 0x40);
  for (uint8_t i = 0; i < length; i++)
  {
    *packet++ = Rf_Spi_Read_Byte();
  }
  RFChip_Disable;
}

void A7105_WriteID(uint32_t ida)
{
  RFChip_Enable;
  Rf_Spi_Write_Byte(A7105_REG_ID_DATA);
  Rf_Spi_Write_Byte((ida >> 24) & 0xff);
  Rf_Spi_Write_Byte((ida >> 16) & 0xff);
  Rf_Spi_Write_Byte((ida >> 8) & 0xff);
  Rf_Spi_Write_Byte((ida >> 0) & 0xff);

  RFChip_Disable;
}

uint32_t A7105_ReadID(void)
{
  uint8_t data[4];

  RFChip_Enable;
  Rf_Spi_Write_Byte(A7105_REG_ID_DATA | 0x40);
  data[0] = Rf_Spi_Read_Byte();
  data[1] = Rf_Spi_Read_Byte();
  data[2] = Rf_Spi_Read_Byte();
  data[3] = Rf_Spi_Read_Byte();
  RFChip_Disable;
  return ((data[0] << 24) | (data[1] << 16) | (data[2] << 8) | (data[3] << 0));
}
