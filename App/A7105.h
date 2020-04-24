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

#ifndef __A7105_H__
#define __A7105_H__

#include "stm32f0xx.h"

#define A7105_REG_MODE 0x00
#define A7105_REG_MODE_CONTROL 0x01
#define A7105_REG_CALC 0x02
#define A7105_REG_FIFOI 0x03
#define A7105_REG_FIFOII 0x04
#define A7105_REG_FIFO_DATA 0x05
#define A7105_REG_ID_DATA 0x06
#define A7105_REG_RC_OSC_I 0x07
#define A7105_REG_RC_OSC_II 0x08
#define A7105_REG_RC_OSC_III 0x09
#define A7105_REG_CK0_PIN 0x0A
#define A7105_REG_GPIO1_PIN_I 0x0B
#define A7105_REG_GPIO2_PIN_II 0x0C
#define A7105_REG_CLOCK 0x0D
#define A7105_REG_DATA_RATE 0x0E
#define A7105_REG_CHANNEL 0x0F
#define A7105_REG_PLL_I 0x0F
#define A7105_REG_PLL_II 0x10
#define A7105_REG_PLL_III 0x11
#define A7105_REG_PLL_IV 0x12
#define A7105_REG_PLL_V 0x13
#define A7105_REG_TX_I 0x14
#define A7105_REG_TX_II 0x15
#define A7105_REG_DELAY_I 0x16
#define A7105_REG_DELAY_II 0x17
#define A7105_REG_RX 0x18
#define A7105_REG_RX_GAIN_I 0x19
#define A7105_REG_RX_GAIN_II 0x1A
#define A7105_REG_RX_GAIN_III 0x1B
#define A7105_REG_RX_GAIN_IV 0x1C
#define A7105_REG_RSSI_THOLD 0x1D
#define A7105_REG_ADC 0x1E
#define A7105_REG_CODE_I 0x1F
#define A7105_REG_CODE_II 0x20
#define A7105_REG_CODE_III 0x21
#define A7105_REG_IF_CALIB_I 0x22
#define A7105_REG_IF_CALIB_II 0x23
#define A7105_REG_VCO_CURCAL 0x24
#define A7105_REG_VCO_SBCAL_I 0x25
#define A7105_REG_VCO_SBCAL_II 0x26
#define A7105_REG_BATTERY_DET 0x27
#define A7105_REG_TX_TEST 0x28
#define A7105_REG_RX_DEM_TEST_I 0x29
#define A7105_REG_RX_DEM_TEST_II 0x2A
#define A7105_REG_CPC 0x2B
#define A7105_REG_XTAL_TEST 0x2C
#define A7105_REG_PLL_TEST 0x2D
#define A7105_REG_VCO_TEST_I 0x2E
#define A7105_REG_VCO_TEST_II 0x2F
#define A7105_REG_IFAT 0x30
#define A7105_REG_RSCALE 0x31
#define A7105_REG_FILTER_TEST 0x32

#define A7105_SLEEP 0x80
#define A7105_IDLE 0x90
#define A7105_STANDBY 0xA0
#define A7105_PLL 0xB0
#define A7105_RX 0xC0
#define A7105_TX 0xD0
#define A7105_RST_WRPTR 0xE0
#define A7105_RST_RDPTR 0xF0

void A7105_Reset(void);
void A7105_Strobe(uint8_t strobe);
void A7105_WriteReg(uint8_t reg, uint8_t data);
uint8_t A7105_ReadReg(uint8_t reg);
void A7105_WriteFIFO(uint8_t *packet, uint8_t length);
void A7105_ReadFIFO(uint8_t *packet, uint8_t length);
void A7105_WriteID(uint32_t ida);
uint32_t A7105_ReadID(void);

#endif
