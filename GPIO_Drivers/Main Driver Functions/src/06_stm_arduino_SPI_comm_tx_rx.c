/*
 * 06_stm_arduino_SPI_comm_tx_rx.c
 *
 *  Created on: 07-Dec-2019
 *      Author: ashwani
 */



//PB12 as SPI2_NSS in AF5
//PB13 as SPI2_SCK in AF5
//PB15 as SPI2_MOSI in AF5
//PB14 as SPI2_MISO in AF5

#include "stm32f446xx.h"
#include <string.h>


//Command Code
#define COMMAND_LED_CTRL							0x50
#define COMMAND_SENSOR_READ							0x51
#define COMMAND_LED_READ							0x52
#define COMMAND_PRINT								0x53
#define COMMAND_ID_READ								0x54

#define LED_ON										0x1
#define LED_OFF										0x0

//Arduino Analog Pins
#define ANALOG_PIN0									0x0
#define ANALOG_PIN1									0x1
#define ANALOG_PIN2									0x2
#define ANALOG_PIN3									0x3
#define ANALOG_PIN4									0x4
#define ANALOG_PIN5									0x5

//Arduino LED
#define LED_PIN										0x9

uint8_t SPI_VerifyResponse(uint8_t ackbyte)
{
	return ackbyte == 0xF5 ? 1 : 0;
}


void delay(void)
{
	for(uint32_t i = 0; i < 500000/2; i++);
}
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
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PIN_PU;			// As external arduino is connected
	SPIPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;


	//NSS
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_12;
	GPIO_Init(&SPIPins);

	//SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	GPIO_Init(&SPIPins);

	//MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_14;
	GPIO_Init(&SPIPins);

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
	SPI2Handle.SPIConfig.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8;
	SPI2Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT_DFF;
	SPI2Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPIConfig.SPI_SSM = SPI_SSM_DI;						//Hardware Slave management enable for NSS Pin

	SPI_Init(&SPI2Handle);
}

void GPIO_btnInit(void)
{
	GPIO_Handle_t gpiobtn, GpioLed;

	gpiobtn.pGPIOx = GPIOC;
	gpiobtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_13;
	gpiobtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
	gpiobtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	gpiobtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;					//AT OUTPUT THERE IS NO PULL UP NO PULL DOWN RESISTANCE IS CONFIGURED

	GPIO_Init(&gpiobtn);

	//this is led gpio configuration
	GpioLed.pGPIOx = GPIOA;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_5;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;


	GPIO_Init(&GpioLed);
}

int main(void)
{
	uint8_t dummy_write = 0xFF, dummy_read;
	uint8_t commandCode, ackbyte;
	uint8_t analog_read, led_status;
	uint8_t message[] = "Hello ! How are you ??";
	uint8_t id[11];
	uint32_t i=0;

	//Array to pass the arguments
	uint8_t args[2];

	//Initialize the button
	GPIO_btnInit();

	//This function is used to initialize the GPIO Pins to behave like as SPI2 Pins
	SPI2_GPIOInits();

	//This function used to initialize the SPI2 Peripheral Parameter
	SPI2_Inits();

	/*
	 * Making the SSOE 1 does NSS output enable
	 * The NSS pin is automatically managed by the hardware.
	 * i.e. when SPE = 1, NSS will be pulled to low
	 * and NSS pin will be high when SPE = 0
	 */
	SPI_SSOEconfig(SPI2, ENABLE);


	while(1)
	{
		//Wait till button is pressed
		while(GPIO_ReadFromInPin(GPIOC, GPIO_PIN_13));
		delay();

		//1. CMD_LED_CTRL	<pin no(1)>		<value(1)>
		commandCode = COMMAND_LED_CTRL;
		SPI_SendData(SPI2, &commandCode, 1);

		/*
		 *	In SPI communication when master or slave sends 1 byte, it also receives 1 byte
		 *	in return. Therefore transmission of command code of 1 byte resulted 1 garbage byte
		 *	collection in RX buffer of the master and RXNE flag is set. So do the Dummy Read and
		 *	clear the flag.
		 */
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		/*	Let send some dummy data to extract the information for slave shift register
		 *	Slave will only send the response when content of its shift register is shift
		 *	sending data by master
		 */
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte,1);

		/*
		 * This function will take the acknowledge bit(ack bit) receive from the slave and
		 * compare it with ACK and NACK to find what it received
		 */
		if (SPI_VerifyResponse(ackbyte))
		{
			//Send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;

			//Send the argument
			SPI_SendData(SPI2, args, 2);
		}







		//2. CMD_SENSOR_READ	<analog pin number>

		//Wait till button is pressed
		while(GPIO_ReadFromInPin(GPIOC, GPIO_PIN_13));
		delay();

		commandCode = COMMAND_SENSOR_READ;
		SPI_SendData(SPI2, &commandCode, 1);

		/*
		 *	In SPI communication when master or slave sends 1 byte, it also receives 1 byte
		 *	in return. Therefore transmission of command code of 1 byte resulted 1 garbage byte
		 *	collection in RX buffer of the master and RXNE flag is set. So do the Dummy Read and
		 *	clear the flag.
		 */
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		/*	Let send some dummy data to extract the information for slave shift register
		 *	Slave will only send the response when content of its shift register is shift
		 *	sending data by master
		 */
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte,1);

		/*
		 * This function will take the acknowledge bit(ack bit) receive from the slave and
		 * compare it with ACK and NACK to find what it received
		 */
		if (SPI_VerifyResponse(ackbyte))
		{
			//Send arguments
			args[0] = ANALOG_PIN0;

			//Send the argument
			SPI_SendData(SPI2, args, 1);

			//Dummy read to clear the garbage value
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//Insert some delay so that slave can ready with data after ADC conversion
			delay();

			//Send some dummy bits(1 byte) in order to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			//Read the response of the command
			SPI_ReceiveData(SPI2, &analog_read, 1);
		}





		//3. COMMAND_LED_READ	<pin no>

		//Wait till button is pressed
		while(GPIO_ReadFromInPin(GPIOC, GPIO_PIN_13));
		delay();

		commandCode = COMMAND_LED_READ;
		SPI_SendData(SPI2, &commandCode, 1);

		/*
		 *	In SPI communication when master or slave sends 1 byte, it also receives 1 byte
		 *	in return. Therefore transmission of command code of 1 byte resulted 1 garbage byte
		 *	collection in RX buffer of the master and RXNE flag is set. So do the Dummy Read and
		 *	clear the flag.
		 */
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		/*	Let send some dummy data to extract the information for slave shift register
		 *	Slave will only send the response when content of its shift register is shift
		 *	sending data by master
		 */
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte,1);

		/*
		 * This function will take the acknowledge bit(ack bit) receive from the slave and
		 * compare it with ACK and NACK to find what it received
		 */
		if (SPI_VerifyResponse(ackbyte))
		{
			//Send arguments
			args[0] = LED_PIN;

			//Send the argument
			SPI_SendData(SPI2, args, 1);

			//Dummy read to clear the garbage value
			SPI_ReceiveData(SPI2, &dummy_read, 1);

			//Insert some delay so that slave can ready with data after ADC conversion
			delay();

			//Send some dummy bits(1 byte) in order to fetch the response from the slave
			SPI_SendData(SPI2, &dummy_write, 1);

			//Read the response of the command
			SPI_ReceiveData(SPI2, &led_status, 1);
		}





		//4. CMD_PRINT 		<len(2)>  <message(len) >

		//Wait till button is pressed
		while(GPIO_ReadFromInPin(GPIOC, GPIO_PIN_13));
		delay();


		commandCode = COMMAND_PRINT;
		SPI_SendData(SPI2, &commandCode, 1);

		/*
		 *	In SPI communication when master or slave sends 1 byte, it also receives 1 byte
		 *	in return. Therefore transmission of command code of 1 byte resulted 1 garbage byte
		 *	collection in RX buffer of the master and RXNE flag is set. So do the Dummy Read and
		 *	clear the flag.
		 */
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		/*	Let send some dummy data to extract the information for slave shift register
		 *	Slave will only send the response when content of its shift register is shift
		 *	sending data by master
		 */
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte,1);

		/*
		 * This function will take the acknowledge bit(ack bit) receive from the slave and
		 * compare it with ACK and NACK to find what it received
		 */
		if (SPI_VerifyResponse(ackbyte))
		{
			//Send arguments
			args[0] = strlen((char*)message);

			//Send the argument, length of the message to be send first before actual data
			SPI_SendData(SPI2, args, 1);


			//Send message
			SPI_SendData(SPI2, message, args[0]);

		}




		//5. CMD_ID_READ

		//Wait till button is pressed
		while(GPIO_ReadFromInPin(GPIOC, GPIO_PIN_13));
		delay();

		commandCode = COMMAND_ID_READ;
		SPI_SendData(SPI2, &commandCode, 1);

		/*	In SPI communication when master or slave sends 1 byte, it also receives 1 byte
		 *	in return. Therefore transmission of command code of 1 byte resulted 1 garbage byte
		 *	collection in RX buffer of the master and RXNE flag is set. So do the Dummy Read and
		 *	clear the flag.
		 */
		SPI_ReceiveData(SPI2, &dummy_read, 1);

		/*	Let send some dummy data to extract the information for slave shift register
		 *	Slave will only send the response when content of its shift register is shift
		 *	sending data by master
		 */
		SPI_SendData(SPI2, &dummy_write, 1);

		//Read the ack byte received
		SPI_ReceiveData(SPI2, &ackbyte,1);

		/*
		 * This function will take the acknowledge bit(ack bit) receive from the slave and
		 * compare it with ACK and NACK to find what it received
		 */
		if (SPI_VerifyResponse(ackbyte))
		{
			//read 10 bytes id from the slave
			for(  i = 0 ; i < 10 ; i++)
			{
				//send dummy byte to fetch data from slave
				SPI_SendData(SPI2,&dummy_write,1);
				SPI_ReceiveData(SPI2,&id[i],1);
			}

			id[11] = '\0';
		}


		//1st check SPI is not busy before closing the peripheral
		while(SPI_GetFlagStatus(SPI2, SPI_BSY_FLAG));


		//Disable the SPI2 Peripheral
		SPI_peripheralControl(SPI2, DISABLE);

	}

	return 0;
}
