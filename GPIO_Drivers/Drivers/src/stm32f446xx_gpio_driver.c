/*
 * stm32f466xx_gpio_driver.c
 *
 *  Created on: 26-Nov-2019
 *      Author: ashwani
 */

#include "stm32f446xx_gpio_driver.h"


/*************************************************************************
 *@function Name			-	GPIO_PeriClkCntl()
 *
 *@Brief					-	This function enable the clock for the given GPIO register
 *
 *@param[in]				-	Base address of the GPIO peripheral
 *@param[in]				-	ENABLE(1) or DISABLE Macro(2)
 *
 *
 *@return					-	VOID
 *
 *@Note						-
 */
void GPIO_PeriClkCntl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)//if user want to ENABLE the Clock
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		}
	}
	else
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}
}


/*************************************************************************
 *@function Name		-	GPIO_Init
 *
 *@Brief				-	This function deals with user defined initialization the GPIO PIN to I/O,
 *							speed of operation Push/Pull configuration, Pull up/down resistance and
 *							alternative function of PIN
 *@param[in]			- 	Base address of the pGPIOHandle variable which hold all the functionality
 *							to be set on the PIN
 *
 *
 *
 *@return				-	VOID
 *
 *@Note					-
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	//Enable the GPIOx Clock
	GPIO_PeriClkCntl(pGPIOHandle->pGPIOx, ENABLE);


	uint32_t modeSet = 0; 																								// to set the pin modes in GPIO register
	uint8_t altFnRegister;																								// 'O' means first 8 pins in ALTL register and 1 means 9 - 16 pins in ALTH register
	uint8_t altFnBit;																									// Multiplying altFnBit with 4 will give the Bit position

	uint8_t extiRegisterSelect;																							// to select one of the four EXTI register by PinNumber/4
	uint8_t extiGPIOPinselect;																							// to select the pin in the EXTI register by PinNumber%4
	uint8_t portCode;
	//1. Configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) //Non-Interrupt mode
	{
		modeSet = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);								//Clear the value
		pGPIOHandle->pGPIOx->MODER |= modeSet;																			//Set the value
 	}

	else //Interrupt mode
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_FT)	// Rising edge interrupt
		{
			//1. Configure the FTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//Clear the corresponding RTSR BIT
			EXTI->RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RT) // Falling edge interrupt
		{
			//1. Configure the RTSR
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

			//Clear the corresponding FTRT Bit
			EXTI->FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IN_RFT)
		{
			//1. Configure both FISR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

		}

		//2. Configure the GPIO port selection in SYSCFG_EXTICR
		extiRegisterSelect = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		extiGPIOPinselect = pGPIOHandle->GPIO_PinConfig.GPIO_PinMode % 4;
		portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);

		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[extiRegisterSelect] |= portCode << (extiGPIOPinselect * 4);

		//3. Enable the EXTI interrupt delivery using IMR (Interrupt mask register)
			EXTI->IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	modeSet = 0;
	//2. Configure the speed
	modeSet = pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);								//Clear the value
	pGPIOHandle->pGPIOx->OSPEEDER |= modeSet;																			//Set the value
	modeSet = 0;

	//3. Configure the Push/Pull setting
	modeSet = pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);									//Clear the value
	pGPIOHandle->pGPIOx->PUPDR |= modeSet;																				//Set the value
	modeSet = 0;

	//4. Configure the output type
	modeSet = pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);								//Clear the value
	pGPIOHandle->pGPIOx->OTYPER |= modeSet;																				//set the value

	//5. Configure the Alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction == GPIO_MODE_ALTFN)
	{
		altFnRegister = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8);
		altFnBit = (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8);

		pGPIOHandle->pGPIOx->AFR[altFnRegister] &= (0xF << ( 4 * altFnBit));											//Clear the value
		pGPIOHandle->pGPIOx->AFR[altFnRegister] |= pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunction << ( 4 * altFnBit);	//Set the value
	}
}


/*************************************************************************
 *@function Name			-	PIO_DeInit
 *
 *@Brief					-	RESET the GPIO register to its initial status
 *
 *@param[in]				-	Base address of the GPIO register
 *@param[in]				-
 *
 *
 *@return					-	VOID
 *
 *@Note						-
 */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{

	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}

}


/*************************************************************************
 *@function Name			-	GPIO_ReadFromInPin
 *
 *@Brief					-	This function read the value of the individual PIN from the GPIO Register
 *
 *@param[in]				-	Base address to the GPIO Register from which particular PIN value to read
 *@param[in]				-	Pin number
 *
 *
 *@return					-	Value store in GPIO register for the particular PIN
 *
 *@Note						-
 */
uint8_t GPIO_ReadFromInPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t bit;
	bit = (uint8_t)(pGPIOx->IDR >> PinNumber) & 0x00000001 ;
	return bit;
}


/*************************************************************************
 *@function Name			-	GPIO_ReadFromInPort
 *
 *@Brief					-	Read the Whole GPIO Register
 *
 *@param[in]				-	Base address of the GPIO register whose content has to be read
 *@param[in]				-
 *
 *
 *@return					-	Value stored in the GPIO register
 *
 *@Note						-
 */
uint16_t GPIO_ReadFromInPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}


/*************************************************************************
 *@function Name			-	GPIO_WriteToOutPin
 *
 *@Brief					-	This function write at the particular GPIO Pin
 *
 *@param[in]				-	Base address to the GPIO Register
 *@param[in]				-	Pin Number
 *
 *
 *@return					-	NONE
 *
 *@Note						-
 */
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value)
{
	if(value == GPIO_PIN_SET)//write 1 to the output data register at the corresponding pin number
	{
		pGPIOx->ODR |= (1 << PinNumber);
	}
	else//write 0
	{
		pGPIOx->ODR &= ~(1 << PinNumber);
	}
}


/*************************************************************************
 *@function Name			-	GPIO_WriteToOutPort
 *
 *@Brief					-	This function write to the whole GPIO register
 *
 *@param[in]				-	Base address of the GPIO register whose content has to be modify
 *@param[in]				-
 *
 *
 *@return					-	NONE
 *
 *@Note						-
 */
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}


/*************************************************************************
 *@function Name			-	PIO_ToggleOutputPin
 *
 *@Brief					-	This function toggle the current state of the GPIO PIN
 *
 *@param[in]				-	Base address of the GPIO Register
 *@param[in]				-	PIN Number that has to be toggle
 *
 *
 *@return					-	VOID
 *
 *@Note						-
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= (1 << PinNumber);
}


/*************************************************************************
 *@function Name			-	GPIO_IRQHanding
 *
 *@Brief					-
 *
 *@param[in]				-
 *@param[in]				-
 *
 *
 *@return					-
 *
 *@Note						-
 */
void GPIO_IRQHanding(uint8_t PinNumber)
{
	//Clear the EXTI Pending register corresponding to the pin number
	if(EXTI->PR & (1 << PinNumber))
	{
		//Clear writing one will clear the register
		EXTI->PR |= (1 << PinNumber);
	}
}


/*************************************************************************
 *@function Name			-	GPIO_IRQconfig
 *
 *@Brief					-
 *
 *@param[in]				-
 *@param[in]				-
 *
 *
 *@return					-
 *
 *@Note						-
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	//Enable and Disable NVIC Base on PIN Number
	if(EnorDi == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//Program ISER0 Register
			*NVIC_ISER0 |= (1 << IRQNumber) ;
		}

		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ISER1 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32));
		}

		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ISER2 Register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32));
		}
	}
	else
	{
		if(IRQNumber <= 31)
		{
			//Program ICER0 Register
			*NVIC_ICER0 |= (1 << IRQNumber) ;
		}

		else if(IRQNumber > 31 && IRQNumber < 64)
		{
			//Program ICER1 Register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}

		else if(IRQNumber >= 64 && IRQNumber < 96)
		{
			//Program ICER2 Register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32));
		}
	}
}


/*************************************************************************
 *@function Name			-	IRQPriorityConfig
 *
 *@Brief					-
 *
 *@param[in]				-
 *@param[in]				-
 *
 *
 *@return					-
 *
 *@Note						-
 */
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
{
	//1. Find out the IPR Register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BIT_IMPLEMENTED); // as first four bit of the register are not implemented therefore we have to shift 4 bit extra
	*(NVIC_PR_BASEADDR + (iprx)) |= (IRQPriority << (shift_amount));
}
