/*
 * stm32446xx.h
 *
 *  Created on: 25-Nov-2019
 *      Author: ashwani
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_
#include <stdint.h>
#include <string.h>
#include <stddef.h>

//Some Generic Macro
#define __VO 										volatile
#define ENABLE										1
#define DISABLE										0
#define	SET											ENABLE
#define RESET										DISABLE
#define	GPIO_PIN_SET								SET
#define GPIO_PIN_RESET								RESET
#define FLAG_SET									SET
#define FLAG_RESET									RESET
#define __weak										__attribute__((weak))


/***************************************** START: PROCESSOR SPECIFIC DETAILS ***********************************************/
/*
 * ARM Cortex M4 Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0									( (__VO uint32_t *) 0xE000E100 )
#define NVIC_ISER1									( (__VO uint32_t *) 0xE000E104 )
#define NVIC_ISER2									( (__VO uint32_t *) 0xE000E108 )
#define NVIC_ISER3									( (__VO uint32_t *) 0xE000E10C )

/*
 * ARM Cortex M4 Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0									( (__VO uint32_t *) 0XE000E180 )
#define NVIC_ICER1									( (__VO uint32_t *) 0XE000E184 )
#define NVIC_ICER2									( (__VO uint32_t *) 0XE000E188 )
#define NVIC_ICER3									( (__VO uint32_t *) 0XE000E18C )

/*
 * ARM Cortex M4 Processor NVIC PR Base Register Addresses
 */
//#define NVIC_PR_BASEADDR							0xE000E400UL
#define NVIC_PR_BASEADDR							( (__VO uint32_t *) 0xE000E400 )

#define NO_PR_BIT_IMPLEMENTED						4

/*
 * IRQ(Interrupt request) Number of STM32F446xx MCU
*/
#define IRQ_NO_EXTI0								6
#define IRQ_NO_EXTI1								7
#define IRQ_NO_EXTI2								8
#define IRQ_NO_EXTI3								9
#define IRQ_NO_EXTI4								10
#define IRQ_NO_EXTI9_5								23
#define IRQ_NO_EXTI15_10							40
#define IRQ_NO_SPI1									35
#define IRQ_NO_SPI2									36
#define IRQ_NO_SPI3									51
#define IRQ_NO_I2C1_EV								38
#define IRQ_NO_I2C1_ER								39
#define IRQ_NO_I2C2_EV								40
#define IRQ_NO_I2C2_ER								41
#define IRQ_NO_I2C3_EV								79
#define IRQ_NO_I2C3_ER								80


/*
 * Macro for all the possible IRQ PRIORITY levels
 */
#define NVIC_IRQ_PRI0								13
#define NVIC_IRQ_PRI1								14
#define NVIC_IRQ_PRI2								15
#define NVIC_IRQ_PRI3								16
#define NVIC_IRQ_PRI4								17
#define NVIC_IRQ_PRI9_5								30
#define NVIC_IRQ_PRI15_10							47
#define NVIC_IRQ_SPI1								42
#define NVIC_IRQ_SPI2								43
#define NVIC_IRQ_SPI3								58
#define NVIC_IRQ_I2C1_EV							31
#define NVIC_IRQ_I2C1_ER							32
#define NVIC_IRQ_I2C2_EV							33
#define NVIC_IRQ_I2C2_ER							34
#define NVIC_IRQ_I2C3_EV							72
#define NVIC_IRQ_I2C3_ER							73
/*****************************************PERIPHERAL REGISTER DEFINITION STRUCTURE************************************************/

//GPIO Register
typedef struct
{
	__VO uint32_t MODER;		/*!< GPIO port mode register,									Address offset: 0x00 */
	__VO uint32_t OTYPER;		/*!< GPIO port output type register,							Address offset: 0x04 */
	__VO uint32_t OSPEEDER;		/*!< GPIO port output speed register,							Address offset: 0x08 */
	__VO uint32_t PUPDR;		/*!< GGPIO port pull-up/pull-down register,						Address offset: 0x0C */
	__VO uint32_t IDR;			/*!< GPIO port input data register,								Address offset: 0x10 */
	__VO uint32_t ODR;			/*!< GPIO port output data register,							Address offset: 0x14 */
	__VO uint32_t BSRR;			/*!< GPIO port bit set/reset register,							Address offset: 0x18 */
	__VO uint32_t LCKR;			/*!< GPIO port configuration lock register,						Address offset: 0x1C */
	__VO uint32_t AFR[2];		/*!< AFR[0] GPIO alternate function Low registers,				Address offset: 0x20 */
								/*!< AFR[1] GPIO alternate function High registers,				Address offset: 0x24 */

}GPIO_RegDef_t;

typedef struct
{
	__VO uint32_t CR;            /*!< RCC clock control register,                                  Address offset: 0x00 */
	__VO uint32_t PLLCFGR;       /*!< RCC PLL configuration register,                              Address offset: 0x04 */
	__VO uint32_t CFGR;          /*!< RCC clock configuration register,                            Address offset: 0x08 */
	__VO uint32_t CIR;           /*!< RCC clock interrupt register,                                Address offset: 0x0C */
	__VO uint32_t AHB1RSTR;      /*!< RCC AHB1 peripheral reset register,                          Address offset: 0x10 */
	__VO uint32_t AHB2RSTR;      /*!< RCC AHB2 peripheral reset register,                          Address offset: 0x14 */
	__VO uint32_t AHB3RSTR;      /*!< RCC AHB3 peripheral reset register,                          Address offset: 0x18 */
	uint32_t      RESERVED0;     /*!< Reserved, 0x1C                                                                    */
	__VO uint32_t APB1RSTR;      /*!< RCC APB1 peripheral reset register,                          Address offset: 0x20 */
	__VO uint32_t APB2RSTR;      /*!< RCC APB2 peripheral reset register,                          Address offset: 0x24 */
	uint32_t      RESERVED1[2];  /*!< Reserved, 0x28-0x2C                                                               */
	__VO uint32_t AHB1ENR;       /*!< RCC AHB1 peripheral clock register,                          Address offset: 0x30 */
	__VO uint32_t AHB2ENR;       /*!< RCC AHB2 peripheral clock register,                          Address offset: 0x34 */
	__VO uint32_t AHB3ENR;       /*!< RCC AHB3 peripheral clock register,                          Address offset: 0x38 */
	uint32_t      RESERVED2;     /*!< Reserved, 0x3C                                                                    */
	__VO uint32_t APB1ENR;       /*!< RCC APB1 peripheral clock enable register,                   Address offset: 0x40 */
	__VO uint32_t APB2ENR;       /*!< RCC APB2 peripheral clock enable register,                   Address offset: 0x44 */
	uint32_t      RESERVED3[2];  /*!< Reserved, 0x48-0x4C                                                               */
	__VO uint32_t AHB1LPENR;     /*!< RCC AHB1 peripheral clock enable in low power mode register, Address offset: 0x50 */
	__VO uint32_t AHB2LPENR;     /*!< RCC AHB2 peripheral clock enable in low power mode register, Address offset: 0x54 */
	__VO uint32_t AHB3LPENR;     /*!< RCC AHB3 peripheral clock enable in low power mode register, Address offset: 0x58 */
	uint32_t      RESERVED4;     /*!< Reserved, 0x5C                                                                    */
	__VO uint32_t APB1LPENR;     /*!< RCC APB1 peripheral clock enable in low power mode register, Address offset: 0x60 */
	__VO uint32_t APB2LPENR;     /*!< RCC APB2 peripheral clock enable in low power mode register, Address offset: 0x64 */
	uint32_t      RESERVED5[2];  /*!< Reserved, 0x68-0x6C                                                               */
	__VO uint32_t BDCR;          /*!< RCC Backup domain control register,                          Address offset: 0x70 */
	__VO uint32_t CSR;           /*!< RCC clock control & status register,                         Address offset: 0x74 */
	uint32_t      RESERVED6[2];  /*!< Reserved, 0x78-0x7C                                                               */
	__VO uint32_t SSCGR;         /*!< RCC spread spectrum clock generation register,               Address offset: 0x80 */
	__VO uint32_t PLLI2SCFGR;    /*!< RCC PLLI2S configuration register,                           Address offset: 0x84 */
	__VO uint32_t PLLSAICFGR;    /*!< RCC PLLSAI configuration register,                           Address offset: 0x88 */
	__VO uint32_t DCKCFGR;       /*!< RCC Dedicated Clocks configuration register,                 Address offset: 0x8C */
	__VO uint32_t CKGATENR;      /*!< RCC Clocks Gated ENable Register,                            Address offset: 0x90 */
	__VO uint32_t DCKCFGR2;      /*!< RCC Dedicated Clocks configuration register 2,               Address offset: 0x94 */
}RCC_RegDef_t;

typedef struct
{
	__VO uint32_t IMR;					/*!< Interrupt mask register						Address offset:	0x00	>*/
	__VO uint32_t EMR;					/*!< Event mask register							Address offset:	0x04	>*/
	__VO uint32_t RTSR;					/*!< Rising trigger selection register				Address offset:	0x08	>*/
	__VO uint32_t FTSR;					/*!< Falling trigger selection register				Address offset:	0x0C	>*/
	__VO uint32_t SWIR;					/*!< Software interrupt event register				Address offset:	0x10	>*/
	__VO uint32_t PR;					/*!< Pending register								Address offset:	0x14	>*/
}EXTI_RegDef_t;

typedef struct
{
	__VO uint32_t MEMRMP;				/*!< SYSCFG memory remap register											Address offset:	0x00	>*/
	__VO uint32_t PMC;					/*!< SYSCFG peripheral mode configuration register							Address offset:	0x04	>*/
	__VO uint32_t EXTICR[4];			/*!< SYSCFG external interrupt configuration register 1-4					Address offset:	0x08 - 0x14	>*/
	uint32_t RESERVED[2];
	__VO uint32_t CMPCR;				/*!< Compensation cell control register										Address offset:	0x20	>*/
	uint32_t RESERVED2[2];
	__VO uint32_t CFGR;					/*!< SYSCFG configuration register											Address offset:	0x2C	>*/

}SYSCFG_RegDef_t;

typedef struct
{
	__VO uint32_t CR1;					/*!< SPI control register 1 (SPI_CR1) (not used in I2S mode)				Address offset: 0x00 >*/
	__VO uint32_t CR2;					/*!< SPI control register 2 (SPI_CR2)										Address offset: 0x04 >*/
	__VO uint32_t SR;					/*!< SPI status register (SPI_SR)											Address offset: 0x08 >*/
	__VO uint32_t DR;					/*!< SPI data register (SPI_DR)												Address offset: 0x0C >*/
	__VO uint32_t CRCPR;				/*!< SPI CRC polynomial register (SPI_CRCPR) (not used in I2S mode)>		Address offset: 0x10 >*/
	__VO uint32_t RXCRCR;				/*!< SPI RX CRC register (SPI_RXCRCR) (not used in I2S mode)				Address offset: 0x14 >*/
	__VO uint32_t TXCRCR;				/*!< SPI TX CRC register (SPI_TXCRCR) (not used in I2S mode)				Address offset: 0x18 >*/
	__VO uint32_t I2SCFGR;				/*!< SPI_I2S configuration register (SPI_I2SCFGR)							Address offset: 0x1C >*/
	__VO uint32_t I2SPR;				/*!< SPI_I2S prescaler register (SPI_I2SPR)									Address offset: 0x20 >*/

}SPI_RegDef_t;

typedef struct
{
	__VO uint32_t CR1;					/* !<>*/
	__VO uint32_t CR2;					/* !<>*/
	__VO uint32_t OAR1;					/* !<>*/
	__VO uint32_t OAR2;					/* !<>*/
	__VO uint32_t DR;					/* !<>*/
	__VO uint32_t SR1;					/* !<>*/
	__VO uint32_t SR2;					/* !<>*/
	__VO uint32_t CCR;					/* !<>*/
	__VO uint32_t TRISE;				/* !<>*/
	__VO uint32_t FLTR;					/* !<>*/


}I2C_RegDef_t;

//Base address of embedded memories on the MUC
#define FLASH_BASEADDR					0x08000000UL 							//Base address of the FLASH U mean unsigned value
#define SRAM1_BASEADDR					0x20000000UL 							//Base address of the SRAM1
#define SRAM2_BASEADDR					0x2001C000UL 							//Base address of the SRAM2
#define ROM								0x1FFF0000UL							//Base address of the system memory i.e. ROM
#define OTP								0x1FFF7800UL

#define SRAM							SRAM1_BASEADDR

//Base address of the various bus domain on the MCU
#define PERIPH_BASEADDR					0x40000000UL 							/*!< Peripheral base address in the alias region> */
#define APB1PERIPH_BASEADDR				PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR				0x40010000UL
#define AHB1PERIPH_BASEADDR				0x40020000UL
#define AHB2PERIPH_BASEADDR				0x50000000UL

//Define base address of peripheral which are hanging on the AHB1 BUS
#define GPIOA_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0000UL)
#define GPIOB_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0400UL)
#define GPIOC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0800UL)
#define GPIOD_BASEADDR					(AHB1PERIPH_BASEADDR + 0x0C00UL)
#define GPIOE_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1000UL)
#define GPIOF_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1400UL)
#define GPIOG_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1800UL)
#define GPIOH_BASEADDR					(AHB1PERIPH_BASEADDR + 0x1C00UL)
#define RCC_BASEADDR					(AHB1PERIPH_BASEADDR + 0x3800UL)

//Define base address of peripheral which are hanging on the APB1 BUS
#define SPI2_BASEADDR					(APB1PERIPH_BASEADDR + 0x3800UL)
#define SPI3_BASEADDR					(APB1PERIPH_BASEADDR + 0x3C00UL)

#define USART2_BASEADDR					(APB1PERIPH_BASEADDR + 0x4400UL)
#define USART3_BASEADDR					(APB1PERIPH_BASEADDR + 0x4800UL)
#define UART4_BASEADDR					(APB1PERIPH_BASEADDR + 0x4C00UL)
#define UART5_BASEADDR					(APB1PERIPH_BASEADDR + 0x5000UL)

#define I2C1_BASEADDR					(APB1PERIPH_BASEADDR + 0x5400UL)
#define I2C2_BASEADDR					(APB1PERIPH_BASEADDR + 0x5800UL)
#define I2C3_BASEADDR					(APB1PERIPH_BASEADDR + 0x5C00UL)


//Define base address of peripheral which are hanging on the APB2 BUS

#define USART1_BASEADDR					(APB2PERIPH_BASEADDR + 0x1000UL)
#define USART6_BASEADDR					(APB2PERIPH_BASEADDR + 0x1400UL)

#define SPI1_BASEADDR					(APB2PERIPH_BASEADDR + 0x3000UL)
#define SYSCFG_BASEADDR					(APB2PERIPH_BASEADDR + 0x3800UL)
#define EXTI_BASEADDR					(APB2PERIPH_BASEADDR + 0x3C00UL)

/*****************************************PERIPHERAL REGISTER DEFINITION STRUCTURE END*****************************************************/




//Peripheral definition
#define GPIOA										((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB										((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC										((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD										((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE										((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF										((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG										((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH										((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define RCC											((RCC_RegDef_t *)RCC_BASEADDR)
#define EXTI										((EXTI_RegDef_t *)EXTI_BASEADDR)
#define SYSCFG										((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)
#define SPI1										((SPI_RegDef_t *) SPI1_BASEADDR)
#define SPI2										((SPI_RegDef_t *) SPI2_BASEADDR)
#define SPI3										((SPI_RegDef_t *) SPI3_BASEADDR)
#define I2C1										((I2C_RegDef_t *) I2C1_BASEADDR)
#define I2C2										((I2C_RegDef_t *) I2C2_BASEADDR)
#define I2C3										((I2C_RegDef_t *) I2C3_BASEADDR)

/************************************************CLOCK ENABLE MACRO START****************************************************/
/*
 * Clock Enable MACROS for GPIOx Peripherals
 */
#define GPIOA_PCLK_EN()								(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()								(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()								(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()								(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()								(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()								(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()								(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()								(RCC->AHB1ENR |= (1 << 7))

/*
 * Clock Enable MACROS for I2Cx Peripherals
 */
#define I2C1_PCLK_EN()								(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()								(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()								(RCC->APB1ENR |= (1 << 23))

/*
 * Clock Enable MACROS for SPIx Peripherals
 */
#define SPI1_PCLK_EN()								(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()								(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()								(RCC->APB1ENR |= (1 << 15))


/*
 * Clock Enable MACROS for UARTx Peripherals
 */
#define UART4_PCLK_EN()								(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()								(RCC->APB1ENR |= (1 << 20))

/*
 * Clock Enable MACROS for USARTx Peripherals
 */
#define USART2_PCLK_EN()							(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()							(RCC->APB1ENR |= (1 << 18))
#define USART1_PCLK_EN()							(RCC->APB2ENR |= (1 << 4))
#define USART6_PCLK_EN()							(RCC->APB2ENR |= (1 << 5))

/*
 * Clock Enable MACROS for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_EN()							(RCC->APB2ENR |= (1 << 14))


/************************************************CLOCK ENABLE MACRO END****************************************************/




/************************************************CLOCK DISABLE MACRO START****************************************************/
/*
 * Clock Disable MACROS for GPIOx Peripherals
 */
#define GPIOA_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()								(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock Disable MACROS for I2Cx Peripherals
 */
#define I2C1_PCLK_DI()								(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()								(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()								(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock Disable MACROS for SPIx Peripherals
 */
#define SPI1_PCLK_DI()								(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()								(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()								(RCC->APB1ENR &= ~(1 << 15))


/*
 * Clock Disable MACROS for UARTx Peripherals
 */
#define UART4_PCLK_DI()								(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()								(RCC->APB1ENR &= ~(1 << 20))

/*
 * Clock Disable MACROS for USARTx Peripherals
 */
#define USART2_PCLK_DI()							(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()							(RCC->APB1ENR &= ~(1 << 18))
#define USART1_PCLK_DI()							(RCC->APB2ENR &= ~(1 << 4))
#define USART6_PCLK_DI()							(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock Disable MACROS for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_DI()							(RCC->APB2ENR &= ~(1 << 14))


/*************************************************CLOCK DISABLE MACRO END*****************************************************/

/*
 * MACROS to reset GPIOx Peripheral
 * First we have to write 1 we Reset and it is necessary to clear, so that PIN don't sent the continuous rest signal
 */

#define GPIOA_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()							do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)

#define GPIO_BASEADDR_TO_CODE(x)					(	(x == GPIOA) ? 0 :\
														(x == GPIOB) ? 1 :\
														(x == GPIOC) ? 2 :\
														(x == GPIOD) ? 3 :\
														(x == GPIOE) ? 4 :\
														(x == GPIOF) ? 5 :\
														(x == GPIOG) ? 6 :\
														(x == GPIOH) ? 7 : 0 )

/*
 * MACROS to reset SPIx Peripheral
 */

#define SPI1_REG_RESET()							do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); } while(0)
#define SPI2_REG_RESET()							do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB1RSTR &= ~(1 << 14)); } while(0)
#define SPI3_REG_RESET()							do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB1RSTR &= ~(1 << 15)); } while(0)

/*
 * MACROS to reset I2Cx Peripheral
 */
#define I2C1_REG_RESET()							do{ (RCC->APB1RSTR |= (1 << 21)); (RCC->APB1RSTR &= ~(1 << 21)); } while(0)
#define I2C2_REG_RESET()							do{ (RCC->APB1RSTR |= (1 << 22)); (RCC->APB1RSTR &= ~(1 << 22)); } while(0)
#define I2C3_REG_RESET()							do{ (RCC->APB1RSTR |= (1 << 23)); (RCC->APB1RSTR &= ~(1 << 23)); } while(0)


/******************************************************************************************************************************
 * 											Bit Position Definition of SPI Peripheral
 *****************************************************************************************************************************/
/*
 * Bit definition for SPI CR1
 */
#define SPI_CR1_CPHA								0x0
#define SPI_CR1_CPOL								0x1
#define SPI_CR1_MSTR								0x2
#define SPI_CR1_BR									0x3 //Three bit wide
#define SPI_CR1_SPE									0x6
#define SPI_CR1_LSBFIRST							0x7
#define SPI_CR1_SSI									0x8
#define SPI_CR1_SSM									0x9
#define SPI_CR1_RXONLY								0xA
#define SPI_CR1_DFF									0xB
#define SPI_CR1_CRCNEXT								0xC
#define SPI_CR1_CRCEN								0xD
#define SPI_CR1_BIDIOE								0xE
#define SPI_CR1_BIDIMODE							0xF

/*
 * Bit definition for SPI CR2
 */
#define SPI_CR2_RXDMAEN								0x0
#define SPI_CR2_TXDMAEN								0x1
#define SPI_CR2_SSOE								0x2
#define SPI_CR2_FRF									0x4
#define SPI_CR2_ERRIE								0x5
#define SPI_CR2_RXNEIE								0x6
#define SPI_CR2_TXEIE								0x7

/*
 * Bit definition for SPI SR
 */
#define SPI_SR_RXNE									0x0
#define SPI_SR_TXE									0x1
#define SPI_SR_CHSIDE								0x2
#define SPI_SR_UDR									0x3
#define SPI_SR_CRCERR								0x4
#define SPI_SR_MODF									0x5
#define SPI_SR_OVR									0x6
#define SPI_SR_BSY									0x7
#define SPI_SR_FRE									0x8


/******************************************************************************************
 *Bit position definitions of I2C peripheral
 ******************************************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				 0
#define I2C_OAR1_ADD71 				 	 1
#define I2C_OAR1_ADD98  			 	 8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_drive.h"
#include "stm32f446xx_i2c_driver.h"

#endif /* INC_STM32F446XX_H_ */

