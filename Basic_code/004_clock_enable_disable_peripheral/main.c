/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* This code explain about how to enable the clock before talking to any
 * peripheral and if clock is not enable then is not possible to talk to that
 * peripheral.
 *
 * 1) To enable the ADC1 which is connected on the APB2 bus we have to access the APB2ENR register
 *    and enable the bit which enable the clock for ADC1 same is done in line-->50
 * 2) To enable the GPIOA which is connected on the AHB1 bus we have to access the AHB1ENR register
 *    and enable the bit which enable the clock for GPIOa same is done in line-->51
 */


/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f446xx.h"


int main(void)
{
	ADC_TypeDef *pADC;
	RCC_TypeDef *pRRC;
	GPIO_TypeDef *pGPIO;

	pGPIO = GPIOA;//link to the base address of GPIO port A
	pRRC = RCC;//link to base address of the RCC
	pADC = ADC1;//link the base address of the ADC1 to the pADC pointer



	//First enable the clock the access the peripheral register
	pRRC->APB2ENR = pRRC->APB2ENR | (1 << 8);
	pRRC->AHB1ENR = pRRC->AHB1ENR | 1;//GPIOA

	//After clock is enable then access the register
	pADC->CR1 = 0x08;
	pGPIO->PUPDR = 0x22;

	return 0;
}
