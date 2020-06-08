/*
 * 04_spi_tx_testing.c
 *
 *  Created on: 02-Dec-2019
 *      Author: ashwani
 */


//PB12 as SPI2_NSS in AF5
//PB13 as SPI2_SCK in AF5
//PB15 as SPI2_MOSI in AF5
//PB14 as SPI2_MISO in AF5

#include "stm32f446xx.h"
#include <string.h>


/*************************************************************************
 *@function Name			-	SPI2_GPIOInits
 *
 *@Brief					- 	This function activate SPI mode on GPIOB
 *
 *@param[in]				-	NONE
 *@param[in]				-	NONE
 *
 *
 *@return					-	VOID
 *
 *@Note						-
 */
void SPI2_GPIOInits(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx = GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunction = GPIO_AFMODE_AF5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MISO
	//SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	//GPIO_Init(&SPIPins);

	//MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_15;
	GPIO_Init(&SPIPins);
}

/*************************************************************************
 *@function Name			-	SPI2_Inits
 *
 *@Brief					- 	This function will set the SPI operation related value like DFF, CPOL, CPHA
 *
 *@param[in]				-	NONE
 *@param[in]				-	NONE
 *
 *
 *@return					-	VOID
 *
 *@Note						-
 */
void SPI2_Inits(void)
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT_DFF;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_EN;

	SPI_Init(&SPI2Handle);
}


int main(void)
{
	char user_data[] = "Hello World";
	//This function is used to initialize the GPIO Pins to behave like as SPI2 Pins
	SPI2_GPIOInits();

	//This function used to initialize the SPI2 Peripheral Parameter
	SPI2_Inits();

	//This make the NSS signal internally high and avoid MODF error
	SPI_SSIconfig(SPI2, ENABLE);

	//Enable the SPI2 Peripheral
	SPI_peripheralControl(SPI2, ENABLE);

	//To send DATA
	SPI_SendData(SPI2, (uint8_t *)user_data, strlen(user_data));

	//1st check SPI is not busy before closing the peripheral
	while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));

	//Disable the SPI2 Peripheral
	SPI_peripheralControl(SPI2, DISABLE);

	while(1);

	return 0;
}
