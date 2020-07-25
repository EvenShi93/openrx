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

#include "Driver_Output.h"
#include "Driver_Flash.h"

extern uint8_t Rssi;
extern uint32_t Miss_Packet_Cnt;
volatile uint16_t Channel_Output[16] = {1500, 1500, 1000, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500, 1500};
uint8_t sbus_pos = 0;
sbus_msg sbus;
uint8_t ppm_pos = 0;
uint16_t ppm_data[10];

void Port_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, ENABLE);

  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_Level_3;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_6 | GPIO_Pin_9 | GPIO_Pin_10;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_PinAFConfig(GPIOA, GPIO_PinSource4, GPIO_AF_4);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource6, GPIO_AF_1);
  GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_2);
  GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_1);

  TIM_TimeBaseStructure.TIM_Period = 1000;
  TIM_TimeBaseStructure.TIM_Prescaler = 0;
  TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;

  TIM_TimeBaseInit(TIM14, &TIM_TimeBaseStructure);

  TIM_TimeBaseStructure.TIM_Period = 20000;
  TIM_TimeBaseStructure.TIM_Prescaler = 47;

  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);

  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
  TIM_OCInitStructure.TIM_OCNPolarity = TIM_OCNPolarity_Low;
  TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Reset;
  TIM_OCInitStructure.TIM_Pulse = 0;

  TIM_OC1Init(TIM14, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM14, TIM_OCPreload_Enable);
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM3, TIM_OCPreload_Enable);
  TIM_OC4Init(TIM3, &TIM_OCInitStructure);
  TIM_OC4PreloadConfig(TIM3, TIM_OCPreload_Enable);

  if (RF_Para.Flash_Para.Output_Mode == PWM_Mode)
  {
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_2);

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);

    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  }
  else
  {
    USART_InitTypeDef USART_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_1);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

    USART_InitStructure.USART_BaudRate = 100000;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_StopBits = USART_StopBits_2;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Tx;
    USART_Init(USART1, &USART_InitStructure);
    USART_InvPinCmd(USART1, USART_InvPin_Tx, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART1, ENABLE);

    TIM_TimeBaseStructure.TIM_Period = 22000;
    TIM_OCInitStructure.TIM_Pulse = 299;

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_BRK_UP_TRG_COM_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);

    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
  }
  TIM_ARRPreloadConfig(TIM1, ENABLE);
  TIM_CtrlPWMOutputs(TIM1, ENABLE);
  TIM_Cmd(TIM1, ENABLE);

  TIM_ARRPreloadConfig(TIM3, ENABLE);
  TIM_CtrlPWMOutputs(TIM3, ENABLE);
  TIM_Cmd(TIM3, ENABLE);

  TIM_ARRPreloadConfig(TIM14, ENABLE);
  TIM_CtrlPWMOutputs(TIM14, ENABLE);
  TIM_Cmd(TIM14, ENABLE);
}

void Port_Output(void)
{
  static uint32_t output_cnt = 0;

  output_cnt++;
  if (RF_Para.Flash_Para.Output_Mode == PWM_Mode)
  {
    TIM1->CCR3 = Channel_Output[0];
    TIM1->CCR2 = Channel_Output[1];
    TIM3->CCR1 = Channel_Output[2];
    TIM3->CCR4 = Channel_Output[3];
    if (Channel_Output[2] > 2000)
      TIM14->CCR1 = 1000;
    else if (Channel_Output[2] > 1050)
      TIM14->CCR1 = Channel_Output[2] - 1000;
    else
      TIM14->CCR1 = 0;
  }
  else
  {
    TIM3->CCR4 = Channel_Output[2];
    TIM14->CCR1 = Channel_Output[3];
    if (output_cnt % 9 == 0)
    {
      sbus.msg.syncByte = 0x0f;
      sbus.msg.ch0 = Channel_Output[0] * 1.6f - 1408;
      sbus.msg.ch1 = Channel_Output[1] * 1.6f - 1408;
      sbus.msg.ch2 = Channel_Output[2] * 1.6f - 1408;
      sbus.msg.ch3 = Channel_Output[3] * 1.6f - 1408;
      sbus.msg.ch4 = Channel_Output[4] * 1.6f - 1408;
      sbus.msg.ch5 = Channel_Output[5] * 1.6f - 1408;
      sbus.msg.ch6 = Channel_Output[6] * 1.6f - 1408;
      sbus.msg.ch7 = Channel_Output[7] * 1.6f - 1408;
      sbus.msg.ch8 = Channel_Output[8] * 1.6f - 1408;
      sbus.msg.ch9 = Channel_Output[9] * 1.6f - 1408;
      sbus.msg.ch10 = Channel_Output[10] * 1.6f - 1408;
      sbus.msg.ch11 = Channel_Output[11] * 1.6f - 1408;
      sbus.msg.ch12 = Channel_Output[12] * 1.6f - 1408;
      sbus.msg.ch13 = Channel_Output[13] * 1.6f - 1408;
      sbus.msg.ch14 = Channel_Output[14] * 1.6f - 1408;
      sbus.msg.ch15 = (Rssi > 114 ? 2047 : Rssi * 16 + 208);
      sbus.msg.flags = (Miss_Packet_Cnt > 47 ? 8 : 0) | (Miss_Packet_Cnt > 2 ? 4 : 0);
      sbus.msg.endByte = 0;
      sbus_pos = 0;
      USART1->CR1 |= USART_CR1_TXEIE;
    }
    if (output_cnt % 22 == 0)
    {
      uint16_t ppmlength = 22000;
      for (uint8_t i = 0; i < 8; i++)
      {
        ppmlength -= Channel_Output[i];
        ppm_data[i] = Channel_Output[i];
      }
      ppm_data[8] = (Rssi > 100 ? 2000 : Rssi * 10 + 1000);
      ppm_data[9] = ppmlength - ppm_data[8];
    }
  }
}

void TIM1_BRK_UP_TRG_COM_IRQHandler(void)
{
  if ((TIM1->SR & TIM_SR_UIF) && (TIM1->DIER & TIM_DIER_UIE))
  {
    TIM1->SR = ~TIM_SR_UIF;
    if (ppm_pos < 10)
      TIM1->ARR = ppm_data[ppm_pos];
    ppm_pos++;
    if (ppm_pos >= 10)
      ppm_pos = 0;
  }
}

void USART1_IRQHandler(void)
{
  uint32_t status = USART1->ISR;
  if (status & USART_ISR_TXE)
  {
    if (sbus_pos >= 25)
      USART1->CR1 &= ~USART_CR1_TXEIE;
    else
      USART1->TDR = sbus.byte[sbus_pos++];
  }
}