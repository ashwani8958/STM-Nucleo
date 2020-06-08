/*
 * stm32f446xx_spi_drive.c
 *
 *  Created on: 01-Dec-2019
 *      Author: ashwani
 */

#include"stm32f446xx_spi_drive.h"

//Some helper function implementation
static void spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	//2. Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
	{
		//16 BIT DFF
		//1. Load the data in to the DR
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pTxBuffer);//1. load 16 bit of data
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t *)pSPIHandle->pTxBuffer++;//increment the buffer to point next bit
	}
	else
	{
		//8 BIT DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pTxBuffer);//1. load 8 bit of data
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;//increment the buffer to point next bit
	}

	if (!pSPIHandle->TxLen)
	{
		//TxLen is Zero, so close the SPI transmission and inform the application that Tx is over

		//This prevent interrupts from setting up of TXE FLAG
		SPI_CloseTransmission(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_TX_CMPLT);
	}
}


static void spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//2. Check the DFF bit in CR1
	if(pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
	{
		//16 BIT DFF
		pSPIHandle->pSPIx->DR = *((uint16_t *)pSPIHandle->pRxBuffer);
		pSPIHandle->Rxlen--;
		pSPIHandle->Rxlen--;
		(uint16_t *)pSPIHandle->pRxBuffer++;//increment the buffer to point next bit
	}
	else
	{
		//8 BIT DFF
		pSPIHandle->pSPIx->DR = *(pSPIHandle->pRxBuffer);//1. load 8 bit of data
		pSPIHandle->Rxlen--;
		pSPIHandle->pRxBuffer++;//increment the buffer to point next bit
	}

	if (!pSPIHandle->Rxlen)
	{

		//This prevent interrupts from setting up of RXNE FLAG
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_RX_CMPLT);
	}
}


static void spi_ovr_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	uint8_t temp;
	//Clear the Overrun flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX)
	{
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}

	(void) temp;

	//then inform the application
	SPI_ApplicationEventCallback(pSPIHandle, SPI_EVENT_OVR_ERR);

}


/*************************************************************************
 *@function Name			-	SPI_PeriClkCntl
 *
 *@Brief					-	This function will enable the clock for the SPI Peripheral which is necessary before using the SPI
 *
 *@param[in]				-	SPIx register address
 *@param[in]				-	ENABLE or DISABLE MACRO
 *
 *
 *@return					-	NONE
 *
 *@Note						-
 */
void SPI_PeriClkCntl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_EN();
		}
		if(pSPIx == SPI2)
		{
			SPI2_PCLK_EN();
		}
		if(pSPIx == SPI3)
		{
			SPI3_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx == SPI1)
		{
			SPI1_PCLK_DI();
		}
		if(pSPIx == SPI2)
		{
			SPI2_PCLK_DI();
		}
		if(pSPIx == SPI3)
		{
			SPI3_PCLK_DI();
		}
	}
}


/*************************************************************************
 *@function Name			-	SPI_Init
 *
 *@Brief					-	This function will initialize control register 1 for the SPIx Peripheral.
 *
 *@param[in]				-	Take address of the pSPI Handle register
 *@param[in]				-
 *
 *
 *@return					-	NONE
 *
 *@Note						-
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//Peripheral Clock enable
	SPI_PeriClkCntl(pSPIHandle->pSPIx, ENABLE);


	uint32_t tempreg = 0;

	//1. Configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. configure the bus
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDI mode Should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
	}
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDI mode Should be set
		tempreg |= (1 << SPI_CR1_BIDIMODE);
	}
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDI mode Should be cleared
		tempreg &= ~(1 << SPI_CR1_BIDIMODE);
		//RXONLY Bit must be set
		tempreg |= (1 << SPI_CR1_RXONLY);

	}

	//3. Configure the DATA FRAME Length
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//4. Configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;


	//5. Configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	//6. Configure the SPEED of SPI Peripheral
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//7. Configure the SSM mode (software slave management)
	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	//8. Configure the CR1 Register
	pSPIHandle->pSPIx->CR1 = tempreg;

}


/*************************************************************************
 *@function Name			-	SPI_DeInit
 *
 *@Brief					-	This function will RESET the SPIx peripheral to default RESET value
 *
 *@param[in]				-	Base Address of the SPIx peripheral Port
 *@param[in]				-
 *
 *
 *@return					-	None
 *
 *@Note						-
 */
 void SPI_DeInit(SPI_RegDef_t *pSPIx)
 {
	 if(pSPIx == SPI1)
	 {
		 SPI1_REG_RESET();
	 }

	 else if (pSPIx == SPI2)
	 {
		 SPI2_REG_RESET();
	 }
	 else if (pSPIx == SPI2)
	 {
		 SPI2_REG_RESET();
	 }
 }

 /*************************************************************************
  *@function Name			-	SPI_GetFlagStatus
  *
  *@Brief					-	This function will check the flag bit in Status Register and find whether it is set or reset
  *
  *@param[in]				-	SPI register Base address
  *@param[in]				-	SPI FLAG NAME
  *
  *
  *@return					-	FLAG_RESET and FLAG_SET
  *
  *@Note					-
  */
 uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName)
 {
	 if(pSPIx->SR & flagName)
	 {
		 return FLAG_SET;
	 }
	 return FLAG_RESET;
 }

 /*************************************************************************
  *@function Name			-	SPI_SendData
  *
  *@Brief					-	This is used to sent the Data to the outside world through the SPI protocol
  *
  *@param[in]				-	Base Address of the SPIx Peripheral port
  *@param[in]				-	Base Address of the transmission Register.
  *@param[in]				-	Length of the data to be transmitted
  *
  *
  *@return					-	NULL
  *
  *@Note					-	This is blocking call/Polling type code
  */
//Blocking method
 void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len)
 {
	 while(len)
	 {
		 //1. wait until TXE is set
		 while((SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)) == FLAG_RESET);

		 //2. Check the DFF bit in CR1
		 if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		 {
			 //16 BIT DFF
			 //1. Load the data in to the DR
			 pSPIx->DR = *((uint16_t *)pTxBuffer);//1. load 16 bit of data
			 len--;
			 len--;
			 (uint16_t *)pTxBuffer++;//increment the buffer to point next bit
		 }
		 else
		 {
			 //8 BIT DFF
			 pSPIx->DR = *(pTxBuffer);//1. load 8 bit of data
			 len--;
			 pTxBuffer++;//increment the buffer to point next bit

		 }

	 }
 }


 /*************************************************************************
  *@function Name			-	SPI_ReceiveData
  *
  *@Brief					-
  *
  *@param[in]				-
  *@param[in]				-
  *
  *
  *@return					-
  *
  *@Note					-
  */
 void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len)
 {
	 while(len)
	 {
		 //1. wait until RXNE is set
		 while((SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)) == FLAG_RESET);

		 //2. Check the DFF bit in CR1
		 if(pSPIx->CR1 & ( 1 << SPI_CR1_DFF))
		 {
			 //16 BIT DFF
			 //1. Load the data from DR to RX Buffer
			 *((uint16_t *)pRxBuffer) = pSPIx->DR;//1. load 16 bit of data
			 len--;
			 len--;
			 (uint16_t *)pRxBuffer++;//increment the buffer to point next bit
		 }
		 else
		 {
			 //8 BIT DFF
			 *(pRxBuffer) = pSPIx->DR;//
			 len--;
			 pRxBuffer++;//increment the buffer to point next bit

		 }

	 }
 }


 /*************************************************************************
  *@function Name			-	SPI_IRQInterruptConfig
  *
  *@Brief					-
  *
  *@param[in]				-
  *@param[in]				-
  *
  *
  *@return					-
  *
  *@Note					-
  */
 void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
  *@function Name			-	SPI_IRQPriorityConfig
  *
  *@Brief					-
  *
  *@param[in]				-
  *@param[in]				-
  *
  *
  *@return					-
  *
  *@Note					-
  */
 void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority)
 {
	 //1. Find out the IPR Register
	 uint8_t iprx = IRQNumber / 4;
	 uint8_t iprx_section = IRQNumber % 4;

	 uint8_t shift_amount = (8 * iprx_section) + ( 8 - NO_PR_BIT_IMPLEMENTED); // as first four bit of the register are not implemented therefore we have to shift 4 bit extra
	 *(NVIC_PR_BASEADDR + (iprx)) |= (IRQPriority << (shift_amount));
 }


 /*************************************************************************
  *@function Name			-	SPI_IRQHanding
  *
  *@Brief					-
  *
  *@param[in]				-
  *@param[in]				-
  *
  *
  *@return					-
  *
  *@Note					-
  */
 void SPI_IRQHanding(SPI_Handle_t *pHandle)
 {
	 uint8_t temp1, temp2;

	 //1st lets check for TXE
	 temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_TXE);
	 temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE);

	 if(temp1 && temp2)
	 {
		 //HANDLE TXE
		 spi_txe_interrupt_handle(pHandle);
	 }


	 //2. Then check for RXNE
	 temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_RXNE);
	 temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE);

	 if(temp1 && temp2)
	 	 {
	 		 //HANDLE RXNE
	 		 spi_rxne_interrupt_handle(pHandle);
	 	 }

	 //3. Then check for OVR
	 temp1 = pHandle->pSPIx->SR & (1 << SPI_SR_OVR);
	 temp2 = pHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE);

	 if(temp1 && temp2)
	 {
		 //HANDLE OVR
		 spi_ovr_interrupt_handle(pHandle);
	 }
 }


 /*************************************************************************
  *@function Name			-	SPI_peripheralControl
  *
  *@Brief					-	This function is used to Enable/Switch ON the SPIx peripheral, setting clock in RCC for GPIOx Port
  *@Brief						enable the user to use the GPIOx port as the SPIx peripheral, the user must enable SPE bit in CR1 register
  *@Brief						to use SPIx peripheral.
  *
  *@param[in]				-	Base address of the pSPIx peripheral
  *@param[in]				-	ENABLE or DISABLE MACRO
  *
  *
  *@return					-	NULL
  *
  *@Note					-
  */
 void SPI_peripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
 {
	 if (EnorDi == ENABLE)
	 {
		 pSPIx->CR1 |= ENABLE << SPI_CR1_SPE;
	 }
	 else
	 {
		 pSPIx->CR1 &= ~(ENABLE << SPI_CR1_SPE);
	 }
 }



 /*************************************************************************
  *@function Name			-	SPI_SSIconfig
  *
  *@Brief					-	As SSI bit influences NSS state when SSM =1. When SSM enable, for master the NSS signal should be tied to +VCC when not
  *@Brief						used to avoid the MODEF error which happen in multimaster situation. So this function take care of the that.
  *
  *@param[in]				-	Base address of the pSPIx peripheral
  *@param[in]				-	ENABLE or DISABLE MACRO
  *
  *
  *@return					-	NULL
  *
  *@Note					-
  */
 void SPI_SSIconfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
 {
	 if (EnorDi == ENABLE)
	 {
		 pSPIx->CR1 |= ENABLE << SPI_CR1_SSI;
	 }
	 else
	 {
		 pSPIx->CR1 &= ~(ENABLE << SPI_CR1_SSI);
	 }
 }

 /*************************************************************************
  *@function Name			-	SPI_SSOEconfig
  *
  *@Brief					-	For master NSS output will be enabled when the SSOE = 1
  *@Brief						NSS output will be enable(HIGH) when SSOE = 1
  *@Brief						NSS = 0, when SPE = 1 (NSS pulled to low automatically when you enable the peripheral because SSOE is enable)
  *@Brief						NSS = 1, when SPE = 0
  *
  *@param[in]				-	Base address of the pSPIx peripheral
  *@param[in]				-	ENABLE or DISABLE MACRO
  *
  *
  *@return					-	NULL
  *
  *@Note					-
  */
 void SPI_SSOEconfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
 {
	 if (EnorDi == ENABLE)
	 {
		 pSPIx->CR2 |= ENABLE << SPI_CR2_SSOE;
	 }
	 else
	 {
		 pSPIx->CR2 &= ~(ENABLE << SPI_CR2_SSOE);
	 }
 }


 /*
  * Interrupt based send and Receive
  */
 uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len)
 {
	 uint8_t state = pSPIHandle->TxState;
	 if (state != SPI_BUSY_IN_TX)
	 {
		 //1. Save the Tx buffer Address and Length information in some global variable
		 pSPIHandle->pTxBuffer = pTxBuffer;
		 pSPIHandle->TxLen = len;

		 //2. Mark the SPI state as busy in transmission so that no other code can take
		 //	over same SPI peripheral until transmission is over
		 pSPIHandle->TxState = SPI_BUSY_IN_TX;

		 //3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR.
		 pSPIHandle->pSPIx->CR2 |= (SET << SPI_CR2_TXEIE);
	 }

	 //4. DATA transmission will be handled by the ISR Code

	 return state;
 }



 uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len)
 {
	 uint8_t state = pSPIHandle->RxState;
		 if (state != SPI_BUSY_IN_RX)
		 {
			 //1. Save the Tx buffer Address and Length information in some global variable
			 pSPIHandle->pRxBuffer = pRxBuffer;
			 pSPIHandle->Rxlen = len;

			 //2. Mark the SPI state as busy in transmission so that no other code can take
			 //	over same SPI peripheral until transmission is over
			 pSPIHandle->RxState = SPI_BUSY_IN_RX;

			 //3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR.
			 pSPIHandle->pSPIx->CR2 |= (SET << SPI_CR2_RXNEIE);
		 }

		 //4. DATA transmission will be handled by the ISR Code

		 return state;
 }

void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( SET << SPI_CR2_TXEIE);//clear the TXEIE BIT
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;
}

void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( SET << SPI_CR2_RXNEIE);//clear the RXNEIE BIT
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->Rxlen = 0;
	pSPIHandle->RxState = SPI_READY;
}

void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;
}


__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent)
{
	//this is weak implementation the user application may override this function
}
