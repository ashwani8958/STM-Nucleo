/*
 * 03_interrupt.c
 *
 *  Created on: 30-Nov-2019
 *      Author: ashwani
 */

#include "stm32f446xx.h"


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed, gpioButton;

	//Clear the garbage value stored in gpioLed, gpioButton
	memset(&gpioLed, 0, sizeof(gpioLed));
	memset(&gpioButton, 0, sizeof(gpioButton));

	//Configuration for PIN5 of GPIO as the output pin in response to the interrupt
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//Setting the clock to enable the GPIOA and then initialing
	GPIO_PeriClkCntl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);

	//Configuration for PIN10 of GPIO as the interrupt pin
	gpioButton.pGPIOx = GPIOA;
	gpioButton.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_10;
	gpioButton.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN_FT;
	gpioButton.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
	gpioButton.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	GPIO_PeriClkCntl(GPIOA, ENABLE);
	GPIO_Init(&gpioButton);

	//Interrupt configuration
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15_10);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);


	while(1);


	return 0;
}

void EXTI15_10_IRQHandler(void)
{
	//Handle the Interrupt
	delay();
	GPIO_IRQHanding(GPIO_PIN_10);
	GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
}
