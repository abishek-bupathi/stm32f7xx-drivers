/*
 * stm32f767xx.h
 *
 *  Created on: Aug 18, 2020
 *      Author: abishek_bupathi
 */

#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

#define __vo volatile

/***********************************Base address declaration***********************************/

/*
 * Dase addresses of Flash and SRAM memories
 */

#define FLASH_BASEADDR			0x0800 0000U	// setting base address macro for flash memory
#define SRAM1_BASEADDR			0x2002 0000U	// setting base address macro for SRAM1 memory
#define SRAM1_BASEADDR			0x2007 C000U	// setting base address macro for SRAM2 memory
#define ROM						0x1FF0 0000U	// setting base address macro for ROM memory
#define SRAM 					SRAM1_BASEADDR	// setting default SRAM1 base address as default SRAM base address


/*
 * Dase addresses of APBx and AHBx Peripheral
 */

#define PERIPH_BASEADDR				0X4000 0000U
#define APB1PERIPH_BASEADDR			PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR			0x4001 0000U
#define AHB1PERIPH_BASEADDR			0x4002 0000U
#define AHB2PERIPH_BASEADDR			0x5000 0000U
#define AHB3PERIPH_BASEADDR			0x6000 0000U

/*
 * Base addresses of peripherals on AHB1 bus
 */

#define GPIOA_BASEADDR				(AHB1PERIPH_BASEADDR + 0xOOOO)
#define GPIOB_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO4OO)
#define GPIOC_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO8OO)
#define GPIOD_BASEADDR				(AHB1PERIPH_BASEADDR + 0xOCOO)
#define GPIOE_BASEADDR				(AHB1PERIPH_BASEADDR + 0x10OO)
#define GPIOF_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO4OO)
#define GPIOG_BASEADDR				(AHB1PERIPH_BASEADDR + 0xO8OO)
#define GPIOH_BASEADDR				(AHB1PERIPH_BASEADDR + 0x1COO)
#define GPIOI_BASEADDR				(AHB1PERIPH_BASEADDR + 0x2OOO)
#define GPIOJ_BASEADDR				(AHB1PERIPH_BASEADDR + 0x24OO)
#define GPIOK_BASEADDR				(AHB1PERIPH_BASEADDR + 0x28OO)

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

#endif /* INC_STM32F767XX_H_ */
