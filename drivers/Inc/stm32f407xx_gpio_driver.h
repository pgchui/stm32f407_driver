/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Mar 19, 2021
 *      Author: minghao
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * @GPIO_PINNUMBER
 */
#define GPIO_PIN_NO_0				0
#define GPIO_PIN_NO_1				1
#define GPIO_PIN_NO_2				2
#define GPIO_PIN_NO_3				3
#define GPIO_PIN_NO_4				4
#define GPIO_PIN_NO_5				5
#define GPIO_PIN_NO_6				6
#define GPIO_PIN_NO_7				7
#define GPIO_PIN_NO_8				8
#define GPIO_PIN_NO_9				9
#define GPIO_PIN_NO_10				10
#define GPIO_PIN_NO_11				11
#define GPIO_PIN_NO_12				12
#define GPIO_PIN_NO_13				13
#define GPIO_PIN_NO_14				14
#define GPIO_PIN_NO_15				15

/*
 * @GPIO_MODE
 */
#define GPIO_MODE_IN				0
#define GPIO_MODE_OUT				1
#define GPIO_MODE_ALTFN				2
#define GPIO_MODE_ANALOG			3
#define GPIO_MODE_IT_FT				4	// interrupt falling edge triggered
#define GPIO_MODE_IT_RT				5	// interrupt rising edge triggered
#define GPIO_MODE_IT_FRT			6	// interrupt falling and rising edge triggered

/*
 * @GPIO_OPTYPE
 */
#define GPIO_OPTYPE_PP				0	// output type push-pull
#define GPIO_OPTYPE_OD				1 	// output type open-drain

/*
 * @GPIO_SPEED
 */
#define GPIO_SPEED_LOW				0
#define GPIO_SPEED_MEDIUM			1
#define GPIO_SPEED_FAST				2
#define GPIO_SPEED_VERY_FAST		3

/*
 * @GPIO_PUPD
 */
#define GPIO_PUPD_NONE				0	// no pull-up or pull-down
#define GPIO_PUPD_PU				1	// pull-up
#define GPIO_PUPD_PD				2	// pull-down

/*
 * @GPIO_PORTx
 */
#define GPIO_PORTA					0
#define GPIO_PORTB					1
#define GPIO_PORTC					2
#define GPIO_PORTD					3
#define GPIO_PORTE					4
#define GPIO_PORTF					5
#define GPIO_PORTG					6
#define GPIO_PORTH					7
#define GPIO_PORTI					8

typedef struct
{
	uint8_t	GPIO_PinNumber; 		/* pin number: 0-15 @GPIO_PINNUMBER */
	uint8_t GPIO_PinMode;			/* pin mode: @GPIO_MODE
											00: input (reset state)
											01: general purpose output
											10: alternate function
											11: analog
									*/
	uint8_t GPIO_PinSpeed;			/* output speed: @GPIO_SPEED
											00: low speed
											01: medium speed
											10: high speed
											11: very high speed
									*/
	uint8_t GPIO_PinPuPdControl;	/* pull-up & pull-down control: @GPIO_PUPD
											00: no pull-up or pull-down
											01: pull-up
											10: pull-down
											11: reserved
									*/
	uint8_t GPIO_PinOPType;			/* output type: @GPIO_OPTYPE
											0: output push-pull (reset state)
											1: output open-drain
									*/
	uint8_t GPIO_PinAltFunMode; 	/* alternate functions 0-15
									   note: not all numbers are valid
									*/
}GPIO_PinConfig_t;

typedef struct
{
	GPIO_RegDef_t* pGPIOx;		/* base address of the GPIO port */
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/*
 * Peripheral clock setup
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t enOrDi);

/*
 * Init and de-init
 */
void GPIO_Init(GPIO_Handle_t* pGPIOHandler);
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber, uint8_t value);
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t pinNumber);

/*
 * IRQ configuration and ISR handling
 */
void GPIO_IRQInterruptConfig(uint8_t irqNumber, uint8_t enOrDi);
void GPIO_IRQPriorityConfig(uint8_t irqNumber, uint8_t priority);
void GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
