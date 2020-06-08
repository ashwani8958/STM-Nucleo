/*
 * 08_i2c_master_rx_testing.c
 *
 *  Created on: 09-Dec-2019
 *      Author: ashwani
 */



#include<stdio.h>
#include<string.h>
#include "stm32f446xx.h"

#define MY_ADDR 		0x61
#define SLAVE_ADDR  	0x68

I2C_Handle_t I2C1Handle;

uint8_t recv_data[32];

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


/*
 * PB6-> SCL			with alternative functionality	AF4
 * PB7 -> SDA			with alternative functionality	AF4
 */

void I2C1_GPIOInits(void)
{
	GPIO_Handle_t I2CPins;

	I2CPins.pGPIOx = GPIOB;
	I2CPins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	I2CPins.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_OD;
	I2CPins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
	I2CPins.GPIO_PinConfig.GPIO_PinAltFunction = GPIO_AFMODE_AF4;
	I2CPins. GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;

	//scl
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_6;
	GPIO_Init(&I2CPins);


	//sda
	I2CPins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_7;
	GPIO_Init(&I2CPins);


}

void I2C1_Inits(void)
{

	I2C1Handle.pI2Cx = I2C1;
	I2C1Handle.I2C_Config.I2C_AckControl = I2C_ACK_ENABLE;
	I2C1Handle.I2C_Config.I2C_DeviceAddress = MY_ADDR;
	I2C1Handle.I2C_Config.I2C_FMDutyCycle = I2C_FM_DUTY_2;
	I2C1Handle.I2C_Config.I2C_SCLSpeed = I2C_SCL_SPEED_SM;

	I2C_Init(&I2C1Handle);

}

void GPIO_ButtonInit(void)
{
	GPIO_Handle_t gpiobtn,GpioLed;

	//this is btn gpio configuration
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

	uint8_t commandcode, len;

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//enable the i2c peripheral PE = 1
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE = 1
	I2C_ManageAcking(I2C1, I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( GPIO_ReadFromInPin(GPIOC,GPIO_PIN_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();



		//master(Nucleo) sends the command code 0x51 to read the length(1 byte) of the data from the slave(arduino)
		commandcode = 0x51;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);		//1. send the command
		I2C_MasterReceiveData(&I2C1Handle, &len,1, SLAVE_ADDR, I2C_ENABLE_SR);				//2. Receive the length in len variable, as master must know the length for the data before it receive it

		//master send the command code 0x52 to read the complete data from the slave
		commandcode = 0x52;
		I2C_MasterSendData(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR);		//1. send the command
		I2C_MasterReceiveData(&I2C1Handle, recv_data, len, SLAVE_ADDR, I2C_DISABLE_SR);		//2. Receive the data in the recv_data variable


	}

	return 0;
}
