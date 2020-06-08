/*
 * stm32f44xx_gpio_driver.h
 *
 *  Created on: 26-Nov-2019
 *      Author: ashwani
 */

#ifndef INC_STM32F44XX_GPIO_DRIVER_H_
#define INC_STM32F44XX_GPIO_DRIVER_H_


#include "stm32f446xx.h"
#include <stdint.h>



/*
 * GPIO Pin Number
 */
#define GPIO_PIN_0								0x0
#define GPIO_PIN_1								0x1
#define GPIO_PIN_2								0x2
#define GPIO_PIN_3								0x3
#define GPIO_PIN_4								0x4
#define GPIO_PIN_5								0x5
#define GPIO_PIN_6								0x6
#define GPIO_PIN_7								0x7
#define GPIO_PIN_8								0x8
#define GPIO_PIN_9								0x9
#define GPIO_PIN_10								0xA
#define GPIO_PIN_11								0xB
#define GPIO_PIN_12								0xC
#define GPIO_PIN_13								0xD
#define GPIO_PIN_14								0xE
#define GPIO_PIN_15								0xF


/*
 * GPIO PIN Possible mode
 */
#define GPIO_MODE_IN							0x0
#define GPIO_MODE_OUT							0x1
#define GPIO_MODE_ALTFN							0x2
#define GPIO_MODE_ANALOG						0x3
#define GPIO_MODE_IN_FT							0x4 //INPUT Falling Edge Interrupt Macro
#define GPIO_MODE_IN_RT							0x5 //INPUT Raising Edge Interrupt Macro
#define GPIO_MODE_IN_RFT						0x6 //INPUT Falling Edge Interrupt Macro Trigger


/*
 * GPIO PIN Possible output mode
 */
#define GPIO_OP_TYPE_PP							0x0 //Push-Pull
#define GPIO_OP_TYPE_OD							0x1 //Open Drain

/*
 * GPIO PIN Possible output speed
 */
#define GPIO_SPEED_LOW							0x0
#define GPIO_SPEED_MEDIUM						0x1
#define GPIO_SPEED_FAST							0x2
#define GPIO_SPEED_HIGH							0x3

/*
 * GPIO port Possible pull-up/pull-down register
 */
#define GPIO_NO_PUPD							0x0
#define GPIO_PIN_PU								0x1
#define GPIO_PIN_PD								0x2

/*
 * GPIO Alternate function Macros
 */
#define GPIO_AFMODE_AF0							0x0
#define GPIO_AFMODE_AF1							0x1
#define GPIO_AFMODE_AF2							0x2
#define GPIO_AFMODE_AF3							0x3
#define GPIO_AFMODE_AF4							0x4
#define GPIO_AFMODE_AF5							0x5
#define GPIO_AFMODE_AF6							0x6
#define GPIO_AFMODE_AF7							0x7
#define GPIO_AFMODE_AF8							0x8
#define GPIO_AFMODE_AF9							0x9
#define GPIO_AFMODE_AF10						0xA
#define GPIO_AFMODE_AF11						0xB
#define GPIO_AFMODE_AF12						0xC
#define GPIO_AFMODE_AF13						0xD
#define GPIO_AFMODE_AF14						0xE
#define GPIO_AFMODE_AF15						0xF


/*
 * This is a Pin configuration structure for the GPIO PIN
 */
typedef struct
{
	uint8_t GPIO_PinNumber;					/*! <Possible value from @line 15 - 33 > */
	uint8_t GPIO_PinMode;					/*! <Possible value from @line 36 - 45 > */
	uint8_t GPIO_PinSpeed;					/*! <Possible value from @line 54 - 61 > */
	uint8_t GPIO_PinPuPdControl;			/*! <Possible value from @line 62 - 67 > */
	uint8_t GPIO_PinOPType;					/*! <Possible value from @line 48 - 52 > */
	uint8_t GPIO_PinAltFunction;			/*! <Possible value from @line 74 - 89 > */

}GPIO_PinConfig_t;


/*
 * This is a Handle structure for the GPIO PIN
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;												/*! < This hold the base address of the GPIO Port to which the pin belongs> */
	GPIO_PinConfig_t GPIO_PinConfig;									/*! < This hold GPIO pin configuration setting> */
}GPIO_Handle_t;



/*************************************************************************************************************************
 * 										APIs supported by this driver
 *		 				For more information about the APIs check the function definitions
 *************************************************************************************************************************/
/*
 * Peripheral clock Setup
 */
void GPIO_PeriClkCntl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi);			/*! < *pGPIO will point to base address of the GPIO whose
																			clock has to be change and EnorDi will hold the status of the peripheral> */

/*
 * Init and De-init
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);								/*! < Pointer GPIO handler structure is required > */
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);								/*! < RCC Contain a reset register which can be used to reset peripheral in shot,
																			therefore only base address of particular GPIO RCC is required> */

/*
 * DATA Read and write from PIN/PORT
 */
uint8_t GPIO_ReadFromInPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteToOutPort(GPIO_RegDef_t *pGPIOx, uint16_t value);

/*
 * Toggle pin
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * IRQ configuration and handing
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHanding(uint8_t PinNumber);

#endif /* INC_STM32F44XX_GPIO_DRIVER_H_ */
