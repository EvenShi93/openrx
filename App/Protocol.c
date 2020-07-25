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

#include "Protocol.h"
#include "A7105.h"
#include "Driver_Adc.h"
#include "Driver_Button.h"
#include "Driver_Delay.h"
#include "Driver_Flash.h"
#include "Driver_Led.h"
#include "Driver_Output.h"
#include "Driver_Spi.h"
#include "stdio.h"
#include "stdbool.h"
#include "string.h"

uint8_t Hopping_Table[16][16] = {
    {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
    {0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
    {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
    {0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
    {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
    {0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
    {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
    {0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
    {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
    {0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
    {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
    {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
    {0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
    {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
    {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};

para_union_t RF_Para;
uint32_t Now_Time = 0;
uint32_t Last_Time = 0;
uint32_t Miss_Packet_Cnt = 0;
uint8_t Bind_Flag = 0;
uint8_t Bind_Cnt = 0;
uint8_t FHSS_Chan_Cnt = 0;
uint8_t Protocol_Tx_Packet[32];
uint8_t Protocol_Rx_Packet[32];
uint8_t Packet_Length = 32;
uint8_t Rssi;

void Afhds_Init(void)
{
  Miss_Packet_Cnt = 0;
  FHSS_Chan_Cnt = 0;
  ADC1_Init();
  Bind_Flag = Check_Jump();
  memcpy(&RF_Para, (uint8_t *)Rx_Data_Addr, 1024);
  A7105_Reset();
  Delay_Ms(1);
  Afhds_A7105_Init();
  if (RF_Para.Flash_Para.Output_Mode > Output_Mode_Num)
  {
    RF_Para.Flash_Para.Output_Mode = 0;
    Flash_Data_Save();
  }
  if ((Bind_Flag != RESET) || (RF_Para.Flash_Para.Para_Check != 0x4B424A4F))
  {
    Afhds_Bind();
  }
}

void Afhds_Loop(void)
{
  Port_Init();
  A7105_GDO_Init();
  TimeOut_Init();
  while (1)
  {
    if ((Now_Time - Last_Time) > 10)
    {
      if (Bind_Flag == 2)
      {
        Last_Time = Now_Time;
        Rx_Failsafe();
      }
      else if ((Bind_Cnt == 255) && (Bind_Flag == 0))
      {
        Bind_Flag = 1;
        TIM14->DIER &= ~TIM_DIER_UIE;
        EXTI->IMR &= ~EXTI_IMR_MR0;
        Afhds_Bind();
        A7105_GDO_Init();
        TimeOut_Init();
      }
    }
  }
}

void Afhds_A7105_Init(void)
{
  A7105_WriteID(0x5475c52A);
  A7105_WriteReg(A7105_REG_MODE_CONTROL, 0x42);
  A7105_WriteReg(A7105_REG_CALC, 0x00);
  A7105_WriteReg(A7105_REG_FIFOI, 0x14);
  A7105_WriteReg(A7105_REG_FIFOII, 0x00);
  A7105_WriteReg(A7105_REG_RC_OSC_I, 0x00);
  A7105_WriteReg(A7105_REG_RC_OSC_II, 0x00);
  A7105_WriteReg(A7105_REG_RC_OSC_III, 0x00);
  A7105_WriteReg(A7105_REG_CK0_PIN, 0x00);
  A7105_WriteReg(A7105_REG_GPIO1_PIN_I, 0x01);
  A7105_WriteReg(A7105_REG_GPIO2_PIN_II, 0x21);
  A7105_WriteReg(A7105_REG_CLOCK, 0x05);
  A7105_WriteReg(A7105_REG_DATA_RATE, 0x00);
  A7105_WriteReg(A7105_REG_PLL_I, 0x50);
  A7105_WriteReg(A7105_REG_PLL_II, 0x9E);
  A7105_WriteReg(A7105_REG_PLL_III, 0x4B);
  A7105_WriteReg(A7105_REG_PLL_IV, 0x00);
  A7105_WriteReg(A7105_REG_PLL_V, 0x02);
  A7105_WriteReg(A7105_REG_TX_I, 0x16);
  A7105_WriteReg(A7105_REG_TX_II, 0x2B);
  A7105_WriteReg(A7105_REG_DELAY_I, 0x12);
  A7105_WriteReg(A7105_REG_DELAY_II, 0x00);
  A7105_WriteReg(A7105_REG_RX, 0x62);
  A7105_WriteReg(A7105_REG_RX_GAIN_I, 0x80);
  A7105_WriteReg(A7105_REG_RX_GAIN_II, 0x80);
  A7105_WriteReg(A7105_REG_RX_GAIN_III, 0x00);
  A7105_WriteReg(A7105_REG_RX_GAIN_IV, 0x0A);
  A7105_WriteReg(A7105_REG_RSSI_THOLD, 0x32);
  A7105_WriteReg(A7105_REG_ADC, 0xc3);
  A7105_WriteReg(A7105_REG_CODE_I, 0x0F);
  A7105_WriteReg(A7105_REG_CODE_II, 0x13);
  A7105_WriteReg(A7105_REG_CODE_III, 0xC3);
  A7105_WriteReg(A7105_REG_IF_CALIB_I, 0x00);
  A7105_WriteReg(A7105_REG_VCO_CURCAL, 0x00);
  A7105_WriteReg(A7105_REG_VCO_SBCAL_I, 0x00);
  A7105_WriteReg(A7105_REG_VCO_SBCAL_II, 0x3B);
  A7105_WriteReg(A7105_REG_BATTERY_DET, 0x00);
  A7105_WriteReg(A7105_REG_TX_TEST, 0x17);
  A7105_WriteReg(A7105_REG_RX_DEM_TEST_I, 0x47);
  A7105_WriteReg(A7105_REG_RX_DEM_TEST_II, 0x80);
  A7105_WriteReg(A7105_REG_CPC, 0x03);
  A7105_WriteReg(A7105_REG_XTAL_TEST, 0x01);
  A7105_WriteReg(A7105_REG_PLL_TEST, 0x45);
  A7105_WriteReg(A7105_REG_VCO_TEST_I, 0x18);
  A7105_WriteReg(A7105_REG_VCO_TEST_II, 0x00);
  A7105_WriteReg(A7105_REG_IFAT, 0x01);
  A7105_WriteReg(A7105_REG_RSCALE, 0x0F);
  A7105_WriteReg(A7105_REG_CALC, 0x01);
  while (A7105_ReadReg(A7105_REG_CALC) != 0)
    ;
  A7105_ReadReg(A7105_REG_IF_CALIB_I);
  A7105_WriteReg(A7105_REG_VCO_CURCAL, 0x13);
  A7105_WriteReg(A7105_REG_VCO_SBCAL_I, 0x09);
}

void Afhds_Bind(void)
{
  uint8_t button = 0;
  uint32_t Last_Button = 0;
  uint32_t Last_Led = 0;

  A7105_Strobe(A7105_STANDBY);
  A7105_Strobe(A7105_RST_RDPTR);
  A7105_WriteReg(A7105_REG_CHANNEL, 0x00);
  A7105_Strobe(A7105_RX);
  Last_Time = Now_Time;
  while (1)
  {
    if ((GPIOA->IDR & GPIO_IDR_1) == 0)
    {
      if ((button == 1) && ((Now_Time - Last_Button) > 1000))
      {
        button = 0;
        Last_Button = Now_Time;
        RF_Para.Flash_Para.Output_Mode = (RF_Para.Flash_Para.Output_Mode + 1) % Output_Mode_Num;
      }
    }
    else
      button = 1;
    if ((Now_Time - Last_Led) > 200)
    {
      Last_Led = Now_Time;
      Bind_Led_Status();
    }
    if ((GPIOA->IDR & GPIO_IDR_0) == 0)
    {
      if ((A7105_ReadReg(A7105_REG_MODE) & (1 << 5)) != 0)
      {
        A7105_Strobe(A7105_RST_RDPTR);
        A7105_Strobe(A7105_RX);
      }
      else
      {
        A7105_ReadFIFO(&Protocol_Rx_Packet[0], 5);
        if (Protocol_Rx_Packet[0] == 0xAA)
        {
          RF_Para.Flash_Para.Para_Check = 0x4B424A4F;
          RF_Para.Flash_Para.FailSafe_Check = 0xFFFFFFFF;

          for (uint8_t i = 0; i < 4; i++)
          {
            RF_Para.Flash_Para.Rf_Tx_Id[i] = Protocol_Rx_Packet[1 + i];
          }
          uint8_t row = Protocol_Rx_Packet[1] & 0x0F;
          uint8_t offset = ((Protocol_Rx_Packet[1] & 0xF0) >> 4) + 1;
          if (offset > 0x0A)
            offset = 0x0A;
          for (uint8_t i = 0; i < 16; i++)
          {
            RF_Para.Flash_Para.Hopping_Map[i] = Hopping_Table[row][i] - offset;
          }
          break;
        }
        else
        {
          A7105_Strobe(A7105_RST_RDPTR);
          A7105_WriteReg(A7105_REG_CHANNEL, 0x00);
          A7105_Strobe(A7105_RX);
        }
      }
    }
  }
  Flash_Data_Save();
  for (uint8_t i = 0; i < 80; i++)
  {
    RED_LED_ON;
    GREEN_LED_FLICKER;
    Delay_Ms(45);
  }
  RED_LED_OFF;
  GREEN_LED_OFF;
  NVIC_SystemReset();
}

void A7105_SetChan(uint8_t skip, uint8_t num)
{
  EXTI->IMR &= ~EXTI_IMR_MR0;
  FHSS_Chan_Cnt += skip;
  while (FHSS_Chan_Cnt >= num)
    FHSS_Chan_Cnt -= num;
  A7105_Strobe(A7105_STANDBY);
  A7105_Strobe(A7105_RST_RDPTR);
  A7105_WriteReg(A7105_REG_CHANNEL, RF_Para.Flash_Para.Hopping_Map[FHSS_Chan_Cnt]);
  A7105_Strobe(A7105_RX);
  EXTI->IMR |= EXTI_IMR_MR0;
}

void A7105_GDO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  EXTI_InitTypeDef EXTI_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  NVIC_InitStructure.NVIC_IRQChannel = EXTI0_1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x01;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  SYSCFG_EXTILineConfig(EXTI_PortSourceGPIOA, EXTI_PinSource0);
  EXTI_InitStructure.EXTI_Line = EXTI_Line0;
  EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
  EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Falling;
  EXTI_InitStructure.EXTI_LineCmd = ENABLE;
  EXTI_Init(&EXTI_InitStructure);
}

void TimeOut_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM16, ENABLE);
  TIM_DeInit(TIM16);

  NVIC_InitStructure.NVIC_IRQChannel = TIM16_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x02;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseStructure.TIM_Period = 6000;
  TIM_TimeBaseStructure.TIM_Prescaler = 47;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM16, &TIM_TimeBaseStructure);
  TIM_ClearITPendingBit(TIM16, ENABLE);
  TIM_ITConfig(TIM16, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM16, ENABLE);
}

void EXTI0_1_IRQHandler(void)
{
  if ((EXTI->PR & EXTI_PR_PR0) && (EXTI->IMR & EXTI_IMR_MR0))
  {
    EXTI->PR = EXTI_PR_PR0;
    if ((GPIOA->IDR & GPIO_IDR_0) == 0)
    {
      if ((A7105_ReadReg(A7105_REG_MODE) & (1 << 5)) != 0)
      {
        A7105_Strobe(A7105_RST_RDPTR);
        A7105_Strobe(A7105_RX);
      }
      else
      {
        uint8_t Rssi_new = A7105_ReadReg(A7105_REG_RSSI_THOLD);
        A7105_ReadFIFO(&Protocol_Rx_Packet[0], 21);
        if ((Protocol_Rx_Packet[0] == 0x55) && (Protocol_Rx_Packet[1] == RF_Para.Flash_Para.Rf_Tx_Id[0]) && (Protocol_Rx_Packet[2] == RF_Para.Flash_Para.Rf_Tx_Id[1]) && (Protocol_Rx_Packet[3] == RF_Para.Flash_Para.Rf_Tx_Id[2]) && (Protocol_Rx_Packet[4] == RF_Para.Flash_Para.Rf_Tx_Id[3]))
        {
          GREEN_LED_ON;
          RED_LED_OFF;
          Bind_Flag = 2;
          Miss_Packet_Cnt = 0;
          TIM16->DIER &= ~TIM_DIER_UIE;
          TIM16->ARR = 1550;
          TIM16->CNT = 0;
          TIM16->SR = ~TIM_SR_UIF;
          TIM16->DIER |= TIM_DIER_UIE;
          A7105_SetChan(1, 16);

          for (uint8_t i = 0; i < 8; i++)
          {
            Channel_Output[i] = (Protocol_Rx_Packet[5 + (2 * i)] + 256 * Protocol_Rx_Packet[6 + (2 * i)]);
            if (Channel_Output[i] > 2125)
              Channel_Output[i] = 2125;
            if (Channel_Output[i] < 875)
              Channel_Output[i] = 875;
          }
          Rssi_new = 108 - Rssi_new * 0.7f;
          Rssi = (Rssi * 7 + Rssi_new) >> 3;
        }
      }
    }
  }
}

void TIM16_IRQHandler()
{
  if ((TIM16->SR & TIM_SR_UIF) && (TIM16->DIER & TIM_DIER_UIE))
  {
    TIM16->SR = ~TIM_SR_UIF;
    TIM16->DIER &= ~TIM_DIER_UIE;
    TIM16->CNT = 0;
    A7105_SetChan(1, 16);
    Miss_Packet_Cnt++;
    if (Miss_Packet_Cnt > 125)
    {
      if (Bind_Flag == 0)
      {
        Bind_Cnt++;
      }
      TIM16->ARR = 24000;
      TIM16->DIER |= TIM_DIER_UIE;
      GREEN_LED_OFF;
      RED_LED_ON;
      for (uint8_t i = 0; i < 8; i++)
      {
        Channel_Output[i] = RF_Para.Flash_Para.FailSafe[i];
      }
    }
    else
    {
      TIM16->DIER |= TIM_DIER_UIE;
    }
  }
}

void SysTim_Init(void)
{
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM17, ENABLE);

  TIM_DeInit(TIM17);
  NVIC_InitStructure.NVIC_IRQChannel = TIM17_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPriority = 0x03;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  TIM_TimeBaseStructure.TIM_Period = 999;
  TIM_TimeBaseStructure.TIM_Prescaler = 47;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM17, &TIM_TimeBaseStructure);
  TIM_ClearITPendingBit(TIM17, TIM_IT_Update);
  TIM_ITConfig(TIM17, TIM_IT_Update, ENABLE);
  TIM_Cmd(TIM17, ENABLE);
}

void TIM17_IRQHandler()
{
  if ((TIM17->SR & TIM_SR_UIF) && (TIM17->DIER & TIM_DIER_UIE))
  {
    TIM17->SR = ~TIM_SR_UIF;
    Now_Time++;
    if (Bind_Flag == 2)
      Port_Output();
    ReadVol_Func();
  }
}
