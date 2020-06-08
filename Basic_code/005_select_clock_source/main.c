/*
 * this code explain about how to switch between the different clock source
 * using RRC register
 *
 *By default MCU uses the HSI RC 16MHz oscillator clock
 *	clock source are
 * 1) HSE (High speed external)
 * 2) HSI (High speed internal)
 * 3) PLL
 *
 *To change the default system clock we have to follow below 2 steps
 *1) In RCC clock control register enable the clock bit which you want to use as system clock.	(switch on the source)
 *2) Then from RCC clock configuration register select the source.	(select the source)
 *
 *
 *
 */

#include "stm32f446xx.h"

int main(void)
{
	RCC_TypeDef *pRCC;
	*pRCC = RCC;

	//Step 1 (switch on the source)
	pRCC->CR = pRCC->CR |(1 << 16);
	while(!(pRCC->CR & (1 << 17)));//wait until the HSE become stable for that check the 17 bit of the RCC_CR register()

	//Step 2 (select the source)
	pRCC->CFGR = pRCC->CFGR & ~(0x3);//clear the last two bit
	pRCC->CFGR = pRCC->CFGR | 0x1;//set and select the HSE clock source

	//Check the HSE clock on MSO2
	pRCC->CFGR = pRCC->CFGR | ~(0x7FFFFFFF);

	return 0;

}
