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

#define PERIPH_BASEADDR				0X4000 0000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x4001 0000U
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

// Peripheral definitions (Peripheral base addresses typecasted to xxx_RegDef_t)

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


// Generic Macros

#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

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


#endif /* INC_STM32F767XX_H_ */
