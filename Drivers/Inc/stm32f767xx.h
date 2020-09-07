/*
 * stm32f767xx.h
 *
 *  Created on: Aug 18, 2020
 *      Author: abishek_bupathi
 */

#include <stdint.h>

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#define __vo volatile


/**********************************Processor Specific Details**********************************/

/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */
#define NVIC_ISER0					((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1					((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2					((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3					((__vo uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */
#define NVIC_ICER0					((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1					((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2					((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3					((__vo uint32_t*)0xE000E18C)

/*
 * ARM Cortex Mx Processor Priority Address
 */
#define NVIC_PR_BASE_ADDR 			((__vo uint32_t*)0xE000E400)


#define NO_PR_BITS_IMPLEMENTED 		4

/***********************************Base address declaration***********************************/

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x0800 0000U	// setting base address macro for flash memory
#define SRAM1_BASEADDR			0x2002 0000U	// setting base address macro for SRAM1 memory
#define SRAM2_BASEADDR			0x2007 C000U	// setting base address macro for SRAM2 memory
#define ROM						0x1FF0 0000U	// setting base address macro for ROM memory
#define SRAM 					SRAM1_BASEADDR	// setting default SRAM1 base address as default SRAM base address


/*
 * Base addresses of APBx and AHBx Peripheral
 */

#define PERIPH_BASEADDR				0X40000000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x40010000U
#define AHB1PERIPH_BASEADDR			0x40020000U
#define AHB2PERIPH_BASEADDR			0x5000 0000U
#define AHB3PERIPH_BASEADDR			0x6000 0000U

/*
 * Base addresses of peripherals on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0C00)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0400)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0x0800)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1C00)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2000)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2400)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2800)

#define RCC_BASEADDR				(AHB1PERIPH_BASEADDR + 0X3800)

/*
 * Base addresses of peripherals on APB1 bus
 */

// I2C
#define I2C1_BASEADDR				(APB1PERIPH_BASEADDR + 0x5400)
#define I2C2_BASEADDR				(APB1PERIPH_BASEADDR + 0x5800)
#define I2C3_BASEADDR				(APB1PERIPH_BASEADDR + 0x5C00)
#define I2C4_BASEADDR				(APB1PERIPH_BASEADDR + 0x6000)

// SPI on APB1 bus
#define SPI2_BASEADDR				(APB1PERIPH_BASEADDR + 0x3800)
#define SPI3_BASEADDR				(APB1PERIPH_BASEADDR + 0x3C00)

// USART and UART on APB1 bus
#define USART2_BASEADDR				(APB1PERIPH_BASEADDR + 0x4400)
#define USART3_BASEADDR				(APB1PERIPH_BASEADDR + 0x4800)
#define UART4_BASEADDR				(APB1PERIPH_BASEADDR + 0x4C00)
#define UART5_BASEADDR				(APB1PERIPH_BASEADDR + 0x5000)
#define UART7_BASEADDR				(APB1PERIPH_BASEADDR + 0x7800)
#define UART8_BASEADDR				(APB1PERIPH_BASEADDR + 0x7C00)

/*
 * Base addresses of peripherals on APB2 bus
 */

// SPI
#define SPI1_BASEADDR				(APB2PERIPH_BASEADDR + 0x3000)
#define SPI4_BASEADDR				(APB2PERIPH_BASEADDR + 0x3400)
#define SPI5_BASEADDR				(APB2PERIPH_BASEADDR + 0x5000)
#define SPI6_BASEADDR				(APB2PERIPH_BASEADDR + 0x5400)

// USART on APB2 bus
#define USART1_BASEADDR				(APB2PERIPH_BASEADDR + 0x1000)
#define USART6_BASEADDR				(APB2PERIPH_BASEADDR + 0x1400)

// EXTI
#define EXTI_BASEADDR				(APB2PERIPH_BASEADDR + 0x3C00)

// SYSCFG
#define SYSCFG_BASEADDR				(APB2PERIPH_BASEADDR + 0x3800)


/*************************Peripheral Register definition structures************************/

/*
 * Note: Registers of peripheral are specific to MCU
 */


// Peripheral register definition structure for GPIO
typedef struct{

	__vo uint32_t MODER;		// GPIO port mode register
	__vo uint32_t OTYPER;		// GPIO port output type register
	__vo uint32_t OSPEEDR;		// GPIO port output speed register
	__vo uint32_t PUPDR;		// GPIO port pull-up/pull-down register
	__vo uint32_t IDR;			// GPIO port input data register
	__vo uint32_t ODR;			// GPIO port output data register
	__vo uint32_t BSRR;			// GPIO port bit set/reset register
	__vo uint32_t LCKR;			// GPIO port configuration lock register
	__vo uint32_t AFR[2];		/* AFR[0]: GPIO alternate function low register
								   AFR[1]: GPIO alternate function high register */
}GPIO_RegDef_t;

// Peripheral register definition structure for RCC
typedef struct{

	__vo uint32_t CR;
	__vo uint32_t PLLCFGR;
	__vo uint32_t CFGR;
	__vo uint32_t CIR;
	__vo uint32_t AHB1RSTR;
	__vo uint32_t AHB2RSTR;
	__vo uint32_t AHB3RSTR;
		 uint32_t RESERVED0;
	__vo uint32_t APB1RSTR;
	__vo uint32_t APB2RSTR;
		 uint32_t RESERVED1[2];
	__vo uint32_t AHB1ENR;
	__vo uint32_t AHB2ENR;
	__vo uint32_t AHB3ENR;
		 uint32_t RESERVED2;
	__vo uint32_t APB1ENR;
	__vo uint32_t APB2ENR;
	 	 uint32_t RESERVED3[2];
	__vo uint32_t AHB1LPENR;
	__vo uint32_t AHB2LPENR;
	__vo uint32_t AHB3LPENR;
	 	 uint32_t RESERVED4;
	__vo uint32_t APB1LPENR;
	__vo uint32_t APB2LPENR;
		 uint32_t RESERVED5[2];
	__vo uint32_t BDCR;
	__vo uint32_t CSR;
	 	 uint32_t RESERVED6[2];
	__vo uint32_t SSCGR;
	__vo uint32_t PLLI2SCFGR;
	__vo uint32_t PLLSAICFGR;
	__vo uint32_t DCKCFGR[2];

}RCC_RegDef_t;


// Peripheral register definition structure for EXTI
typedef struct{

	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;

}EXTI_RegDef_t;


// Peripheral register definition structure for SYSCFG
typedef struct{

	__vo uint32_t MEMRMP;
	__vo uint32_t PMC;
	__vo uint32_t EXTICR[4];
	     uint32_t RESERVED1;
	__vo uint32_t CBR;
	__vo uint32_t CMPCR;

}SYSCFG_RegDef_t;


// Peripheral register definition structure for SPI
typedef struct{

	__vo uint32_t CR1;
	__vo uint32_t CR2;
	__vo uint32_t SR;
	__vo uint32_t DR;
	__vo uint32_t CRCPR;
	__vo uint32_t RXCRCR;
	__vo uint32_t TXCRCR;
	__vo uint32_t I2SCFGR;
	__vo uint32_t I2SPR;


}SPI_RegDef_t;

// Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)

#define GPIOA		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH		((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI		((GPIO_RegDef_t*)GPIOI_BASEADDR)
#define GPIOJ		((GPIO_RegDef_t*)GPIOJ_BASEADDR)
#define GPIOK		((GPIO_RegDef_t*)GPIOK_BASEADDR)

#define RCC			((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI		((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG		((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5		((SPI_RegDef_t*)SPI5_BASEADDR)
#define SPI6		((SPI_RegDef_t*)SPI6_BASEADDR)

/****************************Peripheral clock enabling macros***************************/


// Clock enable Macros for GPIOx peripherals

#define GPIOA_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 8))
#define GPIOJ_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 9))
#define GPIOK_PCLK_EN()		(RCC -> AHB1ENR |= (1 << 10))


// Clock enable Macros for I2C peripherals

#define I2C1_PCLK_EN()		(RCC -> APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC -> APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC -> APB1ENR |= (1 << 23))
#define I2C4_PCLK_EN()		(RCC -> APB1ENR |= (1 << 24))


// Clock enable Macros for SPI peripherals

#define SPI1_PCLK_EN()		(RCC -> APB2ENR |= (1 << 21))
#define SPI2_PCLK_EN()		(RCC -> APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC -> APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN()		(RCC -> APB2ENR |= (1 << 13))
#define SPI5_PCLK_EN()		(RCC -> APB2ENR |= (1 << 20))
#define SPI6_PCLK_EN()		(RCC -> APB2ENR |= (1 << 21))


// Clock enable Macros for USART and UART peripherals

#define USART1_PCLK_EN()	(RCC -> APB2ENR |= (1 << 4))
#define UART2_PCLK_EN()		(RCC -> APB1ENR |= (1 << 17))
#define UART3_PCLK_EN()		(RCC -> APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC -> APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC -> APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC -> APB2ENR |= (1 << 5))
#define UART7_PCLK_EN()		(RCC -> APB1ENR |= (1 << 30))
#define UART8_PCLK_EN()		(RCC -> APB1ENR |= (1 << 31))


// Clock enable Macros for SYSCFG peripherals

#define SYSCFG_PCLK_EN()	(RCC -> APB2ENR |= (1 << 14))


/****************************Peripheral clock disabling macros***************************/



// Clock disable Macros for GPIOx peripherals

#define GPIOA_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 8))
#define GPIOJ_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 9))
#define GPIOK_PCLK_DI()		(RCC -> AHB1ENR &= ~(1 << 10))


// Clock disable Macros for I2C peripherals

#define I2C1_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 23))
#define I2C4_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 24))


// Clock disable Macros for SPI peripherals

#define SPI1_PCLK_DI()		(RCC -> APB2ENR &= ~(1 << 21))
#define SPI2_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI()		(RCC -> APB2ENR &= ~(1 << 13))
#define SPI5_PCLK_DI()		(RCC -> APB2ENR &= ~(1 << 20))
#define SPI6_PCLK_DI()		(RCC -> APB2ENR &= ~(1 << 21))


// Clock disable Macros for USART and UART peripherals

#define USART1_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 4))
#define UART2_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 17))
#define UART3_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC -> APB1ENR &= ~1 << 20))
#define USART6_PCLK_DI()	(RCC -> APB2ENR &= ~(1 << 5))
#define UART7_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 30))
#define UART8_PCLK_DI()		(RCC -> APB1ENR &= ~(1 << 31))


// Clock enable Macros for SYSCFG peripherals

#define SYSCFG_PCLK_EN()	(RCC -> APB2ENR |= (1 << 14))


// Macros to reset GPIOx peripheral
#define GPIOA_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define GPIOB_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define GPIOC_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define GPIOD_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define GPIOE_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define GPIOF_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define GPIOG_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define GPIOH_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define GPIOI_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); } while(0)
#define GPIOJ_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 9)); (RCC->AHB1RSTR &= ~(1 << 9)); } while(0)
#define GPIOK_REG_RESET()			do{ (RCC->AHB1RSTR |= (1 << 10)); (RCC->AHB1RSTR &= ~(1 << 10)); } while(0)

// Return the port code of the GPIOx base address
#define GPIO_BASEADDR_TO_CODE(x)  ( (x == GPIOA) ? 0 :\
									(x == GPIOB) ? 1 :\
									(x == GPIOC) ? 2 :\
									(x == GPIOD) ? 3 :\
									(x == GPIOE) ? 4 :\
									(x == GPIOF) ? 5 :\
									(x == GPIOG) ? 6 :\
									(x == GPIOH) ? 7 :\
									(x == GPIOI) ? 8 :\
									(x == GPIOJ) ? 9 :\
									(x == GPIOK) ? 10:0 )

// IRQ Number Macros
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_EXTI5_9				23
#define IRQ_NO_EXTI15_10			40

// IRQ Priority Macros
#define NVIC_PRIORITY_1				1
#define NVIC_PRIORITY_2				2
#define NVIC_PRIORITY_3				3
#define NVIC_PRIORITY_4				4
#define NVIC_PRIORITY_5				5
#define NVIC_PRIORITY_6				6
#define NVIC_PRIORITY_7				7
#define NVIC_PRIORITY_8				8
#define NVIC_PRIORITY_9				9
#define NVIC_PRIORITY_10			10
#define NVIC_PRIORITY_11			11
#define NVIC_PRIORITY_12			12
#define NVIC_PRIORITY_13			13
#define NVIC_PRIORITY_14			14
#define NVIC_PRIORITY_15			15


// Generic Macros

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET
#define FLAG_RESET		RESET
#define FLAG_SET		SET


/***********************************************************************
 * 			Bit position definitions of SPI Peripherals
 **********************************************************************/
// CR1 Register
#define SPI_CR1_CPHA				0
#define SPI_CR1_CPOL				1
#define SPI_CR1_MSTR				2
#define SPI_CR1_BR  				3
#define SPI_CR1_SPE					6
#define SPI_CR1_LSBFIRST			7
#define SPI_CR1_SSI 				8
#define SPI_CR1_SSM 				9
#define SPI_CR1_RXONLY				10
#define SPI_CR1_CRCL				11
#define SPI_CR1_CRCNEXT				12
#define SPI_CR1_CRCEN				13
#define SPI_CR1_BIDIOE				14
#define SPI_CR1_BIDIMODE			15

// CR2 Register
#define SPI_CR2_RXDMAEN				0
#define SPI_CR2_TXDMAEN				1
#define SPI_CR2_SSOE				2
#define SPI_CR2_NASSP				3
#define SPI_CR2_FRF 				4
#define SPI_CR2_ERRIE				5
#define SPI_CR2_RXNEIE				6
#define SPI_CR2_TXEIE   			7
#define SPI_CR2_DS  				8
#define SPI_CR2_FRXTH				12
#define SPI_CR2_LDMA_RX				13
#define SPI_CR2_LDMA_TX				14

// Status Register
#define SPI_SR_RXNE 				0
#define SPI_SR_TXE  				1
#define SPI_SR_CHSIDE				2
#define SPI_SR_UDR      			3
#define SPI_SR_CRCERR				4
#define SPI_SR_MODF					5
#define SPI_SR_OVR					6
#define SPI_SR_BSY  				7
#define SPI_SR_FRE					8
#define SPI_SR_FRLVL				9
#define SPI_SR_FTLVL				11



#include "stm32f767xx_gpio_driver.h"
#include "stm32f767xx_spi_driver.h"

#endif /* INC_STM32F767XX_H_ */
