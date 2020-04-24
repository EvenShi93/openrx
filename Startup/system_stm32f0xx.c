#include "stm32f0xx.h"

uint32_t SystemCoreClock = 48000000;
__I uint8_t AHBPrescTable[16] = {0, 0, 0, 0, 0, 0, 0, 0, 1, 2, 3, 4, 6, 7, 8, 9};

static void SetSysClock(void);

void SystemInit(void)
{
	RCC->CR |= (uint32_t)0x00000001;
	RCC->CFGR &= (uint32_t)0xF8FFB80C;
	RCC->CR &= (uint32_t)0xFEF6FFFF;
	RCC->CR &= (uint32_t)0xFFFBFFFF;
	RCC->CFGR &= (uint32_t)0xFFC0FFFF;
	RCC->CFGR2 &= (uint32_t)0xFFFFFFF0;
	RCC->CFGR3 &= (uint32_t)0xFFFFFEAC;
	RCC->CR2 &= (uint32_t)0xFFFFFFFE;
	RCC->CIR = 0x00000000;
	SetSysClock();
}

void SystemCoreClockUpdate(void)
{
	uint32_t tmp = 0, pllmull = 0, pllsource = 0, prediv1factor = 0;

	tmp = RCC->CFGR & RCC_CFGR_SWS;
	switch (tmp)
	{
	case 0x00:
		SystemCoreClock = HSI_VALUE;
		break;
	case 0x04:
		SystemCoreClock = HSE_VALUE;
		break;
	case 0x08:
		pllmull = RCC->CFGR & RCC_CFGR_PLLMULL;
		pllsource = RCC->CFGR & RCC_CFGR_PLLSRC;
		pllmull = (pllmull >> 18) + 2;
		if (pllsource == 0x00)
		{
			SystemCoreClock = (HSI_VALUE >> 1) * pllmull;
		}
		else
		{
			prediv1factor = (RCC->CFGR2 & RCC_CFGR2_PREDIV1) + 1;
			SystemCoreClock = (HSE_VALUE / prediv1factor) * pllmull;
		}
		break;
	default:
		SystemCoreClock = HSI_VALUE;
		break;
	}
	tmp = AHBPrescTable[((RCC->CFGR & RCC_CFGR_HPRE) >> 4)];
	SystemCoreClock >>= tmp;
}

static void SetSysClock(void)
{
	__IO uint32_t StartUpCounter = 0;

	RCC->CR |= ((uint32_t)RCC_CR_HSEON);
	do
	{
		StartUpCounter++;
	} while (((RCC->CR & RCC_CR_HSERDY) == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));
	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;
		RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLMULL));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_PREDIV1 | RCC_CFGR_PLLXTPRE_PREDIV1_Div2 | RCC_CFGR_PLLMULL12);
		RCC->CR |= RCC_CR_PLLON;
		while ((RCC->CR & RCC_CR_PLLRDY) == 0)
			;
		RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
			;
	}
	else
	{
		RCC->CR &= ~RCC_CR_HSEON;
		RCC->CR |= RCC_CR_HSION;
		FLASH->ACR = FLASH_ACR_PRFTBE | FLASH_ACR_LATENCY;
		RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
		RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE_DIV1;
		RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL));
		RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSI_Div2 | RCC_CFGR_PLLMULL12);
		RCC->CR |= RCC_CR_PLLON;
		while ((RCC->CR & RCC_CR_PLLRDY) == 0)
			;
		RCC->CFGR &= (uint32_t)((uint32_t) ~(RCC_CFGR_SW));
		RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
		while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)RCC_CFGR_SWS_PLL)
			;
	}
}
