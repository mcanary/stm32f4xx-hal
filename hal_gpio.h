

/**
 * @file    hal_gpio.h
 * @brief   STM32F4 GPIO peripheral HAL implementation header file
 */


#ifndef __HAL_GPIO
#define __HAL_GPIO


/************
 * Includes *
 ************/

#include "stm32f4xx.h"


/***********
 * Defines *
 ***********/

/* GPIO Config Field Width Enumerations */
#define HAL_GPIO_MODER_WIDTH   (2)  // MODER config width (2)
#define HAL_GPIO_OTYPER_WIDTH  (1)  // OTYPER config width (1)
#define HAL_GPIO_OSPEEDR_WIDTH (2)  // OSPEEDR config width (2)
#define HAL_GPIO_PUPDR_WIDTH   (2)  // PUPDR config width (2)
#define HAL_GPIO_AFRx_WIDTH    (4)  // AFRx config width (4)
#define HAL_GPIO_IDR_WIDTH     (1)  // IDR field width (1)
#define HAL_GPIO_ODR_WIDTH     (1)  // ODR field width (1)
#define HAL_GPIO_BSRR_WIDTH    (1)  // BSRR field width (1)

/* GPIO_MODER Enumerations */
#define HAL_GPIO_MODER_INPUT  (0x0)  // Input mode
#define HAL_GPIO_MODER_OUTPUT (0x1)  // General-purpose output mode
#define HAL_GPIO_MODER_AF     (0x2)  // Alternate function mode
#define HAL_GPIO_MODER_ANALOG (0x3)  // Analog mode

/* GPIO_OYTPER Enumerations */
#define HAL_GPIO_OTYPER_PUSHPULL  (0x0)  // Push-pull
#define HAL_GPIO_OTYPER_OPENDRAIN (0x1)  // Open-drain

/* GPIO_OSPEEDR Enumerations */
#define HAL_GPIO_OSPEEDR_LOW  (0x0)  // Low-speed mode
#define HAL_GPIO_OSPEEDR_MED  (0x1)  // Medium-speed mode
#define HAL_GPIO_OSPEEDR_FAST (0x2)  // High-speed mode
#define HAL_GPIO_OSPEEDR_HIGH (0x3)  // Very high-speed mode

/* GPIO_PUPDR Enumerations */
#define HAL_GPIO_PUPDR_NONE     (0x0)  // No pull-up/pull-down
#define HAL_GPIO_PUPDR_PULLUP   (0x1)  // Pull-up
#define HAL_GPIO_PUPDR_PULLDOWN (0x2)  // Pull-down

/* GPIO_AFRx Enumerations */
#define HAL_GPIO_AFRx_AF0  (0x0)  // Alternate-function mode 0
#define HAL_GPIO_AFRx_AF1  (0x1)  // Alternate-function mode 1
#define HAL_GPIO_AFRx_AF2  (0x2)  // Alternate-function mode 2
#define HAL_GPIO_AFRx_AF3  (0x3)  // Alternate-function mode 3
#define HAL_GPIO_AFRx_AF4  (0x4)  // Alternate-function mode 4
#define HAL_GPIO_AFRx_AF5  (0x5)  // Alternate-function mode 5
#define HAL_GPIO_AFRx_AF6  (0x6)  // Alternate-function mode 6
#define HAL_GPIO_AFRx_AF7  (0x7)  // Alternate-function mode 7
#define HAL_GPIO_AFRx_AF8  (0x8)  // Alternate-function mode 8
#define HAL_GPIO_AFRx_AF9  (0x9)  // Alternate-function mode 9
#define HAL_GPIO_AFRx_AF10 (0xa)  // Alternate-function mode 10
#define HAL_GPIO_AFRx_AF11 (0xb)  // Alternate-function mode 11
#define HAL_GPIO_AFRx_AF12 (0xc)  // Alternate-function mode 12
#define HAL_GPIO_AFRx_AF13 (0xd)  // Alternate-function mode 13
#define HAL_GPIO_AFRx_AF14 (0xe)  // Alternate-function mode 14
#define HAL_GPIO_AFRx_AF15 (0xf)  // Alternate-function mode 15


/********************
 * Type Definitions *
 ********************/

typedef GPIO_TypeDef hal_gpio_Port_t;
typedef uint8_t hal_gpio_Pin_t;
typedef uint8_t hal_gpio_Mode_t;
typedef uint8_t hal_gpio_OutputType_t;
typedef uint8_t hal_gpio_OutputSpeed_t;
typedef uint8_t hal_gpio_PullUpPullDown_t;
typedef uint8_t hal_gpio_AlternateFunction_t;

typedef struct hal_gpio_Config_s
{
    hal_gpio_Mode_t              Mode;               // GPIO pin mode
    hal_gpio_OutputType_t        OutputType;         // GPIO pin output type config
    hal_gpio_OutputSpeed_t       OutputSpeed;        // GPIO pin output speed config
    hal_gpio_PullUpPullDown_t    PullUpPullDown;     // GPIO pin pull-up/pull-down config
    hal_gpio_AlternateFunction_t AlternateFunction;  // GPIO pin alternate function config
}hal_gpio_Config_s;


/*******************************
 * Public Function Definitions *
 *******************************/

void hal_gpio_Init(hal_gpio_Port_t *port, hal_gpio_Pin_t pin, hal_gpio_Config_s *config);
bool hal_gpio_Read(hal_gpio_Port_t *port, hal_gpio_Pin_t pin);
void hal_gpio_Write(hal_gpio_Port_t *port, hal_gpio_Pin_t pin, bool data);
void hal_gpio_Toggle(hal_gpio_Port_t *port, hal_gpio_Pin_t pin);


#endif /* __HAL_GPIO */

