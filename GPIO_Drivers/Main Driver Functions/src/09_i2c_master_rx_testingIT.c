/*
 * 09_i2c_master_rx_testingIT.c
 *
 *  Created on: 10-Dec-2019
 *      Author: ashwani
 */

#include<stdio.h>
#include<string.h>
#include "stm32f446xx.h"

//extern void initialise_monitor_handles();

#define MY_ADDR					0x61
#define SLAVE_ADDR  			0x68

//Flag variable
uint8_t rxComplt = RESET;

I2C_Handle_t I2C1Handle;

//rcv buffer
uint8_t rcv_buf[32];



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

	//initialise_monitor_handles();
	//printf("Application is running\n");

	GPIO_ButtonInit();

	//i2c pin inits
	I2C1_GPIOInits();

	//i2c peripheral configuration
	I2C1_Inits();

	//I2C IRQ configurations
	I2C_IRQInterruptConfig(NVIC_IRQ_I2C1_EV,ENABLE);
	I2C_IRQInterruptConfig(NVIC_IRQ_I2C1_ER,ENABLE);

	//enable the i2c peripheral
	I2C_PeripheralControl(I2C1,ENABLE);

	//ack bit is made 1 after PE=1
	I2C_ManageAcking(I2C1,I2C_ACK_ENABLE);

	while(1)
	{
		//wait till button is pressed
		while( GPIO_ReadFromInPin(GPIOA,GPIO_PIN_13) );

		//to avoid button de-bouncing related issues 200ms of delay
		delay();


		//master(Nucleo) sends the command code 0x51 to read the length(1 byte) of the data from the slave(arduino) and wait I2C  Ready
		commandcode = 0x51;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) 	!= I2C_READY);				//1. send the command
		while(I2C_MasterReceiveDataIT(&I2C1Handle, &len, 1, SLAVE_ADDR, I2C_ENABLE_SR)			!= I2C_READY);				//2. Receive the length in len variable, as master must know the length for the data before it receive it



		//master send the command code 0x52 to read the complete data from the slave and wait I2C  Ready
		commandcode = 0x52;
		while(I2C_MasterSendDataIT(&I2C1Handle, &commandcode, 1, SLAVE_ADDR, I2C_ENABLE_SR) 	!= I2C_READY);				//1. send the command
		while(I2C_MasterReceiveDataIT(&I2C1Handle, rcv_buf, len, SLAVE_ADDR, I2C_DISABLE_SR)	!= I2C_READY);				//2. Receive the data in the rcv_buf variable

		rxComplt = RESET;

		//wait till rx completes
		while(rxComplt != SET);


		//rcv_buf[len+1] = '\0';
		//printf("Data : %s",rcv_buf);
		rxComplt = RESET;

	}

	return 0;
}


void I2C1_EV_IRQHandler (void)
{
	I2C_EV_IRQHandling(&I2C1Handle);
}


void I2C1_ER_IRQHandler (void)
{
	I2C_ER_IRQHandling(&I2C1Handle);
}



void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CHandle,uint8_t AppEv)
{
	if(AppEv == I2C_EV_TX_CMPLT)
	{
		//printf("Tx is completed\n");
		;
	}

	else if (AppEv == I2C_EV_RX_CMPLT)
	{
		//printf("Rx is completed\n");
		rxComplt = SET;
	}

	else if (AppEv == I2C_ERROR_AF)
	{
		//printf("Error : Ack failure\n");
		//in master ack failure happens when slave fails to send ack for the byte
		//sent from the master.
		I2C_CloseSendData(pI2CHandle);

		//generate the stop condition to release the bus
		I2C_GenerateStopCondition(I2C1);

		//Hang in infinite loop
		while(1);
	}
}

