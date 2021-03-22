/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Mar 19, 2021
 *      Author: minghao
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Peripheral clock setup
 */

/**
 * @fn void GPIO_PeriClockControl(GPIO_RegDef_t*, uint8_t)
 * @brief This function enables or disables peripheral clock for the given GPIO port
 *
 * @param pGPIOx	base address of the GPIO peripheral
 * @param enOrDi	ENABLE or DISABLE macros
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enOrDi)
{
	if (enOrDi == ENABLE)
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_EN();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_EN();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_EN();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_EN();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_EN();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_EN();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_EN();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_EN();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_EN();
		}
	} else
	{
		if (pGPIOx == GPIOA)
		{
			GPIOA_PCLK_DI();
		} else if (pGPIOx == GPIOB)
		{
			GPIOB_PCLK_DI();
		} else if (pGPIOx == GPIOC)
		{
			GPIOC_PCLK_DI();
		} else if (pGPIOx == GPIOD)
		{
			GPIOD_PCLK_DI();
		} else if (pGPIOx == GPIOE)
		{
			GPIOE_PCLK_DI();
		} else if (pGPIOx == GPIOF)
		{
			GPIOF_PCLK_DI();
		} else if (pGPIOx == GPIOG)
		{
			GPIOG_PCLK_DI();
		} else if (pGPIOx == GPIOH)
		{
			GPIOH_PCLK_DI();
		} else if (pGPIOx == GPIOI)
		{
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Init and de-init
 */

/**
 * @fn void GPIO_Init(GPIO_Handle_t*)
 * @brief this function initialize the GPIO pin based on the configuration
 *
 * @param pGPIOHandler
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandler)
{
	uint32_t temp = 0;
	// 1. configure the mode
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{	// non interrupt mode
		temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandler->pGPIOx->MODER &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandler->pGPIOx->MODER |= temp;
	} else
	{	// interrupt mode
		if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// 1.1. configure FTSR
			EXTI->RTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);	// clear RTSR
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);	// set FTSR
		} else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// 1.1. configure RTSR
			EXTI->FTSR &= ~(1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);	// clear FTSR
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);	// set RTSR
		} else if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT)
		{
			// 1.1. configure FTSR and RTSR
			EXTI->FTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);	// set FTSR
			EXTI->RTSR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);	// set RTSR
		}

		// 1.2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t temp3 = 0; // @GPIO_PORTx
		if (pGPIOHandler->pGPIOx == GPIOA)
		{
			temp3 = GPIO_PORTA;
		} else if (pGPIOHandler->pGPIOx == GPIOB)
		{
			temp3 = GPIO_PORTB;
		} else if (pGPIOHandler->pGPIOx == GPIOC)
		{
			temp3 = GPIO_PORTC;
		} else if (pGPIOHandler->pGPIOx == GPIOD)
		{
			temp3 = GPIO_PORTD;
		} else if (pGPIOHandler->pGPIOx == GPIOE)
		{
			temp3 = GPIO_PORTE;
		} else if (pGPIOHandler->pGPIOx == GPIOF)
		{
			temp3 = GPIO_PORTF;
		} else if (pGPIOHandler->pGPIOx == GPIOG)
		{
			temp3 = GPIO_PORTG;
		} else if (pGPIOHandler->pGPIOx == GPIOH)
		{
			temp3 = GPIO_PORTH;
		} else if (pGPIOHandler->pGPIOx == GPIOI)
		{
			temp3 = GPIO_PORTI;
		}
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] &= ~(0xF << (4 * temp2));
		SYSCFG->EXTICR[temp1] |= (temp3 << (4 * temp2));
		// 1.3. enable the EXTI interrupt delivery using IMR
		EXTI->IMR |= (1 << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	}

	// 2. configure the speed
	temp = 0;
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->OSPEEDR |= temp;

	// 3. configure the pull-up/pull-down setting
	temp = 0;
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->PUPDR &= ~(0x3 << (2 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->PUPDR |= temp;

	// 4. configure the output type
	temp = 0;
	temp = (pGPIOHandler->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandler->pGPIOx->OTYPER &= ~(0x1 << (1 * pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandler->pGPIOx->OTYPER |= temp;

	// 5. configure the alternate function
	if (pGPIOHandler->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint32_t temp1 = pGPIOHandler->GPIO_PinConfig.GPIO_PinNumber / 8;
		uint32_t temp2 = pGPIOHandler->GPIO_PinConfig.GPIO_PinMode % 8;
		pGPIOHandler->pGPIOx->AFR[temp1] &= ~(0xF << (4 * temp2));
		pGPIOHandler->pGPIOx->AFR[temp1] |= (pGPIOHandler->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
	}
}

/**
 * @fn void GPIO_DeInit(GPIO_RegDef_t*)
 * @brief This function de-initialize the GPIO port
 *
 * @param pGPIOx
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx)
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	} else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	} else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	} else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	} else if (pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	} else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	} else if (pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	} else if (pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	} else if (pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
}

/*
 * Data read and write
 */

/**
 * @fn uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t*, uint8_t)
 * @brief this function read a value of a GPIO pin
 *
 * @param pGPIOx	GPIO port
 * @param pinNumber	GPIO pin
 * @return value of the GPIO pin
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> pinNumber) & 0x00000001);
	return value;
}

/**
 * @fn uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t*)
 * @brief this function reads values of a GPIO port
 *
 * @param pGPIOx	GPIO port
 * @return values of all pins at the GPIO port
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx)
{
	uint16_t value;
	value = (uint16_t)(pGPIOx->IDR & 0x0000FFFF);
	return value;
}

/**
 * @fn void GPIO_WriteToOutputPin(GPIO_RegDef_t*, uint8_t, uint8_t)
 * @brief this function writes HIGH or LOW value at a GPIO pin
 *
 * @param pGPIOx	GPIO port of that GPIO pin
 * @param pinNumber	GPIO pin number
 * @param value		High/Low to write to the GPIO pin
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value)
{
	if (value == GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(0x1 << pinNumber);
	} else if (value == GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (0x1 << pinNumber);
	}
}

/**
 * @fn void GPIO_WriteToOutputPort(GPIO_RegDef_t*, uint16_t)
 * @brief this function writes HIGH or LOW values at a GPIO port
 *
 * @param pGPIOx	GPIO port to write
 * @param value		values to write
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value)
{
	pGPIOx->ODR = value;
}

/**
 * @fn void GPIO_ToggleOutputPin(GPIO_RegDef_t*, uint8_t)
 * @brief this function toggles the GPIO pin (Low -> High or High -> Low)
 *
 * @param pGPIOx	GPIO port of that GPIO pin
 * @param pinNumber	GPIO pin number to toggle
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber)
{
	pGPIOx->ODR ^= (0x1 << pinNumber);
}

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t irqNumber, uint8_t enOrDi)
{
	if (enOrDi == ENABLE)
	{
		if (irqNumber <= 31)
		{
			// program ISER0
			*NVIC_ISER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber <= 63)
		{
			// program ISER1
			*NVIC_ISER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber > 63 && irqNumber <= 95)
		{
			// program ISER2
			*NVIC_ISER2 |= (1 << (irqNumber % 64));
		}
	} else if (enOrDi == DISABLE)
	{
		if (irqNumber <= 31)
		{
			// program ICER0
			*NVIC_ICER0 |= (1 << irqNumber);
		} else if (irqNumber > 31 && irqNumber <= 63)
		{
			// program ICER1
			*NVIC_ICER1 |= (1 << (irqNumber % 32));
		} else if (irqNumber > 63 && irqNumber <= 95)
		{
			// program ICER2
			*NVIC_ICER2 |= (1 << (irqNumber % 64));
		}
	}
}

void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint8_t priority)
{
	uint8_t iprx = irqNumber / 4;
	uint8_t iprx_section = irqNumber % 4;

	*(NVIC_IPR_BASS_ADDR + iprx) &= ~(0xF << (8 * iprx_section));
	*(NVIC_IPR_BASS_ADDR + iprx) |= (priority << (8 * iprx_section + (8 - NO_PR_BITS_IMPLEMENTED)));
}

void GPIO_IRQHandling(uint8_t pinNumber)
{
	if (EXTI->PR & (1 << pinNumber))
	{
		EXTI->PR |= (1 << pinNumber); // clear the interrupt
	}
}
