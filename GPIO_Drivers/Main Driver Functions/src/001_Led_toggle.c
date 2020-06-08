/*
 * 001_Led_toggle.c
 *
 *  Created on: 27-Nov-2019
 *      Author: ashwani
 */

#include "stm32f446xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 500000; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed;

	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;					//OUTPUT PIN IN PUSH PULL CONFIGURATION
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;					//AT OUTPUT THERE IS NO PULL UP NO PULL DOWN RESISTANCE IS CONFIGURED


	//GPIO_PeriClkCntl(GPIOA, ENABLE);											//ENABLE the GPIO Register so that it can to initialized
	GPIO_Init(&gpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		delay();
	}
	return 0;
}
