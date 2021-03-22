/*
 * stm32f407xx.h
 *
 *  Created on: Mar 19, 2021
 *      Author: minghao
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

#define NVIC_ISER0				((volatile uint32_t*)0xE000E100)
#define NVIC_ISER1				((volatile uint32_t*)0xE000E104)
#define NVIC_ISER2				((volatile uint32_t*)0xE000E108)
#define NVIC_ISER3				((volatile uint32_t*)0xE000E10C)

#define NVIC_ICER0				((volatile uint32_t*)0XE000E180)
#define NVIC_ICER1				((volatile uint32_t*)0xE000E184)
#define NVIC_ICER2				((volatile uint32_t*)0xE000E188)
#define NVIC_ICER3				((volatile uint32_t*)0xE000E18C)

#define NVIC_IPR_BASS_ADDR		((volatile uint32_t*)0xE000E400)

#define NO_PR_BITS_IMPLEMENTED 	4

/*
 * Base addresses of Flash and SRAM memories
 */

#define FLASH_BASE_ADDR			0x08000000U
#define SRAM1_BASE_ADDR			0x20000000U
#define SRAM2_BASE_ADDR			0x2001C000U
#define ROM_BASE_ADDR			0x1FFF0000U
#define SRAM_BASE_ADDR			SRAM1_BASEADDR

/*
 * Base addresses of AHBx and APBx bus peripherals
 */

#define PERIPH_BASE_ADDR		0x40000000U
#define APB1PERIPH_BASE_ADDR	PERIPH_BASE
#define APB2PERIPH_BASE_ADDR	0x40010000U
#define AHB1PERIPH_BASE_ADDR	0x40020000U
#define AHB2PERIPH_BASE_ADDR	0x50000000U

/*
 * Base addresses of AHB1 peripherals
 */

#define GPIOA_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x0000)
#define GPIOB_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x0400)
#define GPIOC_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x0800)
#define GPIOD_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x0C00)
#define GPIOE_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x1000)
#define GPIOF_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x1400)
#define GPIOG_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x1800)
#define GPIOH_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x1C00)
#define GPIOI_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x2000)
//#define GPIOJ_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x2400)
//#define GPIOK_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x2800)
#define RCC_BASE_ADDR			(AHB1PERIPH_BASE_ADDR + 0x3800)

/*
 * Base addresses of APB1 peripherals
 */

#define I2C1_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x5400)
#define I2C2_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x5800)
#define I2C3_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x5C00)
#define SPI2_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x3800)
#define SPI3_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x3C00)
#define USART2_BASE_ADDR		(APB1PERIPH_BASE_ADDR + 0x4400)
#define USART3_BASE_ADDR		(APB1PERIPH_BASE_ADDR + 0x4800)
#define UART4_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x4C00)
#define UART5_BASE_ADDR			(APB1PERIPH_BASE_ADDR + 0x5000)

/*
 * Base addresses of APB2 peripherals
 */

#define SPI1_BASE_ADDR			(APB2PERIPH_BASE_ADDR + 0x3000)
#define USART1_BASE_ADDR		(APB2PERIPH_BASE_ADDR + 0x1000)
#define USART6_BASE_ADDR		(APB2PERIPH_BASE_ADDR + 0x1400)
#define EXTI_BASE_ADDR			(APB2PERIPH_BASE_ADDR + 0x3C00)
#define SYSCFG_BASE_ADDR		(APB2PERIPH_BASE_ADDR + 0x3800)

/*
 * Peripheral register definition structures
 */

typedef struct
{
	volatile uint32_t MODER; 		/*	GPIO port mode register						offset: 0x00 */
	volatile uint32_t OTYPER;		/*	GPIO port output type register				offset: 0x04 */
	volatile uint32_t OSPEEDR;		/*	GPIO port output speed register				offset: 0x08 */
	volatile uint32_t PUPDR;		/*	GPIO port pull-up/pull-down register		offset: 0x0C */
	volatile uint32_t IDR;			/*	GPIO port input data register				offset: 0x10 */
	volatile uint32_t ODR;			/*	GPIO port output data register				offset: 0x14 */
	volatile uint32_t BSRR;			/*	GPIO port bit set/reset register			offset: 0x18 */
	volatile uint32_t LCKR;			/*	GPIO port configuration lock register		offset: 0x1C */
	volatile uint32_t AFR[2];		/*	GPIO alternate function low register		offset: 0x20
	 	 	 	 	 	 	 			GPIO alternate function high register		offset: 0x24 */
}GPIO_RegDef_t;

typedef struct
{
	volatile uint32_t CR;			/*	RCC clock control register					offset: 0x00 */
	volatile uint32_t PLLCFGR;		/*	RCC PLL configuration register				offset: 0x04 */
	volatile uint32_t CFGR;			/*	RCC clock configuration register			offset: 0x08 */
	volatile uint32_t CIR;			/*	RCC clock interrupt register				offset: 0x0C */
	volatile uint32_t AHB1RSTR;		/*	RCC AHB1 peripheral reset register			offset: 0x10 */
	volatile uint32_t AHB2RSTR;		/*	RCC AHB2 peripheral reset register			offset: 0x14 */
	volatile uint32_t AHB3RSTR;		/*	RCC AHB3 peripheral reset register			offset: 0x18 */
	uint32_t RESERVED0;				/*	Reserved									offset: 0x1C */
	volatile uint32_t APB1RSTR;		/*	RCC APB1 peripheral reset register			offset: 0x20 */
	volatile uint32_t APB2RSTR;		/*	RCC APB2 peripheral reset register			offset: 0x24 */
	uint32_t RESERVED1;				/*	Reserved									offset: 0x28 */
	uint32_t RESERVED2;				/*	Reserved									offset: 0x2C */
	volatile uint32_t AHB1ENR;		/*	RCC AHB1 peripheral clock enable register	offset: 0x30 */
	volatile uint32_t AHB2ENR;		/*	RCC AHB2 peripheral clock enable register	offset: 0x34 */
	volatile uint32_t AHB3ENR;		/*	RCC AHB3 peripheral clock enable register	offset: 0x38 */
	uint32_t RESERVED3;				/*	Reserved									offset: 0x3C */
	volatile uint32_t APB1ENR;		/*	RCC APB1 peripheral clock enable register	offset: 0x40 */
	volatile uint32_t APB2ENR;		/*	RCC APB2 peripheral clock enable register	offset: 0x44 */
	uint32_t RESERVED4;				/*	Reserved									offset: 0x48 */
	uint32_t RESERVED5;				/*	Reserved									offset: 0x4C */
	volatile uint32_t AHB1LPENR;	/*	RCC AHB1 peripheral clock enable in low power mode register
																					offset: 0x50 */
	volatile uint32_t AHB2LPENR;	/*	RCC AHB2 peripheral clock enable in low power mode register
																					offset: 0x54 */
	volatile uint32_t AHB3LPENR;	/*	RCC AHB3 peripheral clock enable in low power mode register
																					offset: 0x58 */
	uint32_t RESERVED6;				/*	Reserved									offset: 0x5C */
	volatile uint32_t APB1LPENR;	/*	RCC APB1 peripheral clock enable in low power mode register
																					offset: 0x60 */
	volatile uint32_t APB2LPENR;	/*	RCC APB2 peripheral clock enable in low power mode register
																					offset: 0x64 */
	uint32_t RESERVED7;				/*	Reserved									offset: 0x68 */
	uint32_t RESERVED8;				/*	Reserved									offset: 0x6C */
	volatile uint32_t BDCR;			/*	RCC Backup domain control register			offset: 0x70 */
	volatile uint32_t CSR;			/*	RCC clock control & status register			offset: 0x74 */
	uint32_t RESERVED9;				/*	Reserved									offset: 0x78 */
	uint32_t RESERVED10;				/*	Reserved									offset: 0x7C */
	volatile uint32_t SSCGR;		/*	RCC spread spectrum clock generation register
																					offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;	/*	RCC PLLI2S configuration register			offset: 0x84 */
}RCC_RegDef_t;

typedef struct
{
	volatile uint32_t IMR;			/*	EXTI Interrupt mask register				offset: 0x00 */
	volatile uint32_t EMR;			/*	EXTI Event mask register					offset: 0x04 */
	volatile uint32_t RTSR;			/*	EXTI Rising trigger selection register		offset: 0x08 */
	volatile uint32_t FTSR;			/*	EXTI Falling trigger selection register		offset: 0x0C */
	volatile uint32_t SWIER;		/*	EXTI Software interrupt event register		offset: 0x10 */
	volatile uint32_t PR;			/*	EXTI Pending register						offset: 0x14 */
}EXTI_RegDef_t;

typedef struct
{
	volatile uint32_t MEMRMP;		/*  SYSCFG memory re-map register				offset: 0x00 */
	volatile uint32_t PMC;			/*  SYSCFG peripheral mode configuration register
																					offset: 0x04 */
	volatile uint32_t EXTICR[4];	/* 	SYSCFG external interrupt configuration register 1
																					offset: 0x08
		   	   	   	   	   	   	   	   	SYSCFG external interrupt configuration register 2
																					offset: 0x0C
									   	SYSCFG external interrupt configuration register 3
																					offset: 0x10
									   	SYSCFG external interrupt configuration register 4
																					offset: 0x14 */
	uint32_t RESERVED0;				/*	Reserved									offset: 0x18 */
	uint32_t RESERVED1;				/*	Reserved									offset: 0x1C */
	volatile uint32_t CMPCR;		/* 	SYSCFG Compensation cell control register	offset: 0x20 */
}SYSCFG_RegDef_t;

/*
 * Peripheral definitions
 */

#define GPIOA					((GPIO_RegDef_t*)GPIOA_BASE_ADDR)
#define GPIOB					((GPIO_RegDef_t*)GPIOB_BASE_ADDR)
#define GPIOC					((GPIO_RegDef_t*)GPIOC_BASE_ADDR)
#define GPIOD					((GPIO_RegDef_t*)GPIOD_BASE_ADDR)
#define GPIOE					((GPIO_RegDef_t*)GPIOE_BASE_ADDR)
#define GPIOF					((GPIO_RegDef_t*)GPIOF_BASE_ADDR)
#define GPIOG					((GPIO_RegDef_t*)GPIOG_BASE_ADDR)
#define GPIOH					((GPIO_RegDef_t*)GPIOH_BASE_ADDR)
#define GPIOI					((GPIO_RegDef_t*)GPIOI_BASE_ADDR)
#define RCC						((RCC_RegDef_t*)RCC_BASE_ADDR)
#define EXTI					((EXTI_RegDef_t*)EXTI_BASE_ADDR)
#define SYSCFG					((SYSCFG_RegDef_t*)SYSCFG_BASE_ADDR)

/*
 * Clock enable macros for GPIOx
 */

#define GPIOA_PCLK_EN()		(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()		(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()		(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()		(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()		(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()		(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()		(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()		(RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()		(RCC->AHB1ENR |= (1 << 8))

/*
 * Clock enable macros for I2Cx
 */

#define I2C1_PCLK_EN()		(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN()		(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN()		(RCC->APB1ENR |= (1 << 23))

/*
 * Clock enable macros for SPIx
 */

#define SPI1_PCLK_EN()		(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN()		(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN()		(RCC->APB1ENR |= (1 << 15))

/*
 * Clock enable macros for U(S)ARTx
 */

#define USART1_PCLK_EN()	(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN()	(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN()	(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN()		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN()		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN()	(RCC->APB2ENR |= (1 << 5))

/*
 * Clock enable macros for SYSCFG
 */

#define SYSCFG_PCLK_EN()	(RCC->APB2ENR |= (1 << 14))

/*
 * Clock disable macros for GPIOx
 */

#define GPIOA_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()		(RCC->AHB1ENR &= ~(1 << 8))

/*
 * Clock disable macros for I2Cx
 */

#define I2C1_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 23))

/*
 * Clock disable macros for SPIx
 */

#define SPI1_PCLK_DI()		(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 15))

/*
 * Clock disable macros for U(S)ARTx
 */

#define USART1_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI()	(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI()		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 5))

/*
 * Clock disable macros for SYSCFG
 */

#define SYSCFG_PCLK_DI()	(RCC->APB2ENR &= ~(1 << 14))

/*
 * Reset macros for GPIOx
 */

#define GPIOA_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));} while(0)
#define GPIOB_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));} while(0)
#define GPIOC_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));} while(0)
#define GPIOD_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));} while(0)
#define GPIOE_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));} while(0)
#define GPIOF_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));} while(0)
#define GPIOG_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));} while(0)
#define GPIOH_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));} while(0)
#define GPIOI_REG_RESET()	do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));} while(0)

/*
 * Interrupt Request Number (IRQ_NO)
 */
#define IRQ_NO_EXTI0		6
#define IRQ_NO_EXTI1		7
#define IRQ_NO_EXTI2		8
#define IRQ_NO_EXTI3		9
#define IRQ_NO_EXTI4		10
#define IRQ_NO_EXTI9_5		23
#define IRQ_NO_EXTI15_10	40


/*
 * Generic macros
 */

#define ENABLE			1
#define DISABLE			0
#define SET				ENABLE
#define RESET			DISABLE
#define GPIO_PIN_SET	SET
#define GPIO_PIN_RESET	RESET

#endif /* INC_STM32F407XX_H_ */
