/*
 * stm32f446xx_spi_drive.h
 *
 *  Created on: 01-Dec-2019
 *      Author: ashwani
 */

#ifndef INC_STM32F446XX_SPI_DRIVE_H_
#define INC_STM32F446XX_SPI_DRIVE_H_


#include "stm32f446xx.h"

/*
 * @SPI DeviceMode
 */
#define SPI_DEVICE_MODE_MASTER									0x1
#define SPI_DEVICE_MODE_SLAVE									0x0

/*
 * @SPI BUSConfig
 */
#define SPI_BUS_CONFIG_FD										0x1
#define SPI_BUS_CONFIG_HD										0x2
#define SPI_BUS_CONFIG_SIMPLEX_RXONLY							0x3

/*
 * @SPI SclkSpeed
 */
#define SPI_SCLK_SPEED_DIV2										0x0
#define SPI_SCLK_SPEED_DIV4										0x1
#define SPI_SCLK_SPEED_DIV8										0x2
#define SPI_SCLK_SPEED_DIV16									0x3
#define SPI_SCLK_SPEED_DIV32									0x4
#define SPI_SCLK_SPEED_DIV64									0x5
#define SPI_SCLK_SPEED_DIV128									0x6
#define SPI_SCLK_SPEED_DIV256									0x7

/*
 * @SPI DATAFRAME LENGTH
 */
#define SPI_DFF_8BIT_DFF										0x0
#define SPI_DFF_16BIT_DFF										0x1

/*
 * @SPI CPOL
 */
#define SPI_CPOL_LOW											0x0
#define SPI_CPOL_HIGH											0x1

/*
* @SPI CPHA
 */
#define SPI_CPHA_LOW											0x0
#define SPI_CPHA_HIGH											0x1

/*
 * @SPI SMM
 */
#define SPI_SSM_DI												0x0
#define SPI_SSM_EN												0x1

/*
 * SPI Related Status Flags Definitions
 */
#define SPI_RXNE_FLAG											(1 << SPI_SR_RXNE)
#define SPI_TXE_FLAG											(1 << SPI_SR_TXE)
#define SPI_CHSIDE_FLAG											(1 << SPI_SR_CHSIDE)
#define SPI_UDR_FLAG											(1 << SPI_SR_UDR)
#define SPI_CRCERR_FLAG											(1 << SPI_SR_CRCERR)
#define SPI_MODF_FLAG											(1 << SPI_SR_MODF)
#define SPI_OVR_FLAG											(1 << SPI_SR_OVR)
#define SPI_BSY_FLAG											(1 << SPI_SR_BSY)
#define SPI_FRE_FLAG											(1 << SPI_SR_FRE)

/*
 * Possible SPI Application States
 */
#define SPI_READY												0x0
#define SPI_BUSY_IN_RX											0x1
#define SPI_BUSY_IN_TX											0x2

/*
 * Possible SPI Application events
 */
#define SPI_EVENT_TX_CMPLT										0x1
#define SPI_EVENT_RX_CMPLT										0x2
#define SPI_EVENT_OVR_ERR										0x3

/*
 * Configuration structure for SPIx Peripheral
 */
typedef struct
{
	uint8_t SPI_DeviceMode;										/* < Possible values from @SPI DeviceMode > */
	uint8_t SPI_BusConfig;										/* < Possible values from @SPI BUSConfig > */
	uint8_t SPI_SclkSpeed;										/* < Possible values from @SPI SclkSpeed > */
	uint8_t SPI_DFF;											/* < Possible values from @SPI DATAFRAME LENGTH > */
	uint8_t SPI_CPOL;											/* < Possible values from @SPI CPOL > */
	uint8_t SPI_CPHA;											/* < Possible values from @SPI CPHA > */
	uint8_t SPI_SSM;											/* < Possible values from @SPI SMM > */
}SPI_Config_t;

/*
 * SPI Handle Structure for SPIx peripheral
 */
typedef struct
{
	SPI_Config_t SPIConfig;
	SPI_RegDef_t *pSPIx;
	uint8_t *pTxBuffer;											/* < To store the application Tx buffer Address > */
	uint8_t *pRxBuffer;											/* < To store the application Rx buffer Address > */
	uint32_t TxLen;												/* < To store the Tx Length > */
	uint32_t Rxlen;												/* < To store the Rx Length > */
	uint8_t TxState;											/* < To store the Tx State > */
	uint8_t RxState;											/* < To store the Tx State > */

}SPI_Handle_t;


/*************************************************************************************************************************
 * 										APIs supported by this driver
 *		 				For more information about the APIs check the function definitions
 *************************************************************************************************************************/
/*
 * Peripheral clock Setup
 */
void SPI_PeriClkCntl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);			/*! < *pSPI will point to base address of the SPI whose
																			clock has to be change and EnorDi will hold the status of the peripheral> */

/*
 * Init and De-init
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);							/*! < Pointer SPI handler structure is required > */
void SPI_DeInit(SPI_RegDef_t *pSPIx);								/*! < RCC Contain a reset register which can be used to reset peripheral in shot,
																			therefore only base address of particular SPI RCC is required> */

/*
 * DATA Send and Receive
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len);

/*
 * Interrupt based send and Receive
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t len);


/*
 * IRQ configuration and handing
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHanding(SPI_Handle_t *pHandle);

/*
 * Other Peripheral control API
 */
void SPI_peripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSIconfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
void SPI_SSOEconfig(SPI_RegDef_t *pSPIx, uint8_t EnorDi);
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint32_t flagName);
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx);
void SPI_CloseTransmission(SPI_Handle_t *pSPIHandle);
void SPI_CloseReception(SPI_Handle_t *pSPIHandle);

/*
 * Application callback
 */
void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle, uint8_t appEvent);

#endif /* INC_STM32F446XX_SPI_DRIVE_H_ */
