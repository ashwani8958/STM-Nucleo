/*
 * 02_LED_button.c
 *
 *  Created on: 28-Nov-2019
 *      Author: ashwani
 */


#include "stm32f446xx.h"


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}

int main(void)
{
	GPIO_Handle_t gpioLed, gpiobtn;

/***********************************************LED CONFIGIRATION**********************************************************/
	gpioLed.pGPIOx = GPIOA;
	gpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	gpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	gpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;					//OUTPUT PIN IN PUSH PULL CONFIGIRATION
	gpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;					//AT OUTPUT THERE IS NO PULL UP NO PULL DOWN RESISTANCE IS CONFIGURED

	//Clock initialization to write on GPIO pin
	GPIO_PeriClkCntl(GPIOA, ENABLE);
	GPIO_Init(&gpioLed);
/***********************************************LED CONFIGIRATION**********************************************************/



/***********************************************SWITCH CONFIGIRATION*******************************************************/
	gpiobtn.pGPIOx = GPIOC;
	gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;					//AT OUTPUT THERE IS NO PULL UP NO PULL DOWN RESISTANCE IS CONFIGURED


	GPIO_PeriClkCntl(GPIOC, ENABLE);
	GPIO_Init(&gpiobtn);
/***********************************************SWITCH CONFIGIRATION*******************************************************/

	while(1)
	{
		if(GPIO_ReadFromInPin(GPIOC, GPIO_PIN_13) == 0)
		{
			delay();
			GPIO_ToggleOutputPin(GPIOA, GPIO_PIN_5);
		}

	}
	return 0;
}
