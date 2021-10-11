

/**
 * @file    hal_gpio.c
 * @brief   STM32F4 GPIO peripheral HAL implementation source file
 */


#include "hal_gpio.h"


/*******************************
 * Private Function Prototypes *
 *******************************/

static inline uint32_t hal_gpio_ShiftPinConfig(uint8_t config, uint8_t config_width, hal_gpio_Pin_t pin);
static inline uint32_t hal_gpio_PinMask(uint8_t config_width, hal_gpio_Pin_t pin);


/********************
 * Public Functions *
 ********************/

/**
 * @brief Configure the specified GPIO pin
 * @param port   Peripheral base address for the GPIO port of interest
 * @param pin    GPIO pin number
 * @param config Pointer to a struct containing the GPIO pin configuration parameters
 */
void hal_gpio_Init(hal_gpio_Port_t *port, hal_gpio_Pin_t pin, hal_gpio_Config_s *config)
{

    MODIFY_REG(port->MODER, hal_gpio_PinMask(HAL_GPIO_MODER_WIDTH, pin), \
                            hal_gpio_ShiftPinConfig((uint8_t)config->Mode, HAL_GPIO_MODER_WIDTH, pin));
    MODIFY_REG(port->OTYPER, hal_gpio_PinMask(HAL_GPIO_OTYPER_WIDTH, pin), \
                             hal_gpio_ShiftPinConfig((uint8_t)config->OutputType, HAL_GPIO_OTYPER_WIDTH, pin));
    MODIFY_REG(port->OSPEEDR,hal_gpio_PinMask(HAL_GPIO_OSPEEDR_WIDTH, pin), \
                              hal_gpio_ShiftPinConfig((uint8_t)config->OutputSpeed, HAL_GPIO_OSPEEDR_WIDTH, pin));
    MODIFY_REG(port->PUPDR, hal_gpio_PinMask(HAL_GPIO_PUPDR_WIDTH, pin), \
                            hal_gpio_ShiftPinConfig((uint8_t)config->PullUpPullDown, HAL_GPIO_PUPDR_WIDTH, pin));
    if(pin < 0x8)
    {

        MODIFY_REG(port->AFR[0], hal_gpio_PinMask(HAL_GPIO_AFRx_WIDTH, pin), \
                                  hal_gpio_ShiftPinConfig((uint8_t)config->AlternateFunction, HAL_GPIO_AFRx_WIDTH, pin));

    }
    else
    {

        MODIFY_REG(port->AFR[1], hal_gpio_PinMask(HAL_GPIO_AFRx_WIDTH, pin - 8), \
                                  hal_gpio_ShiftPinConfig((uint8_t)config->AlternateFunction, HAL_GPIO_AFRx_WIDTH, pin - 8));

    }

}

/**
 * @brief Read the specified GPIO pin
 * @param port Peripheral base address for the GPIO port of interest
 * @param pin  GPIO pin number
 * @return State of the specified GPIO pin
 */
bool hal_gpio_Read(hal_gpio_Port_t *port, hal_gpio_Pin_t pin)
{

    if((port->MODER >> (pin * 2)) & 0x3)  // If pin configured as input
    {

        return READ_BIT(port->IDR, hal_gpio_PinMask(HAL_GPIO_IDR_WIDTH, pin));

    }
    else
    {

        return READ_BIT(port->ODR, hal_gpio_PinMask(HAL_GPIO_IDR_WIDTH, pin));

    }

}

/**
 * @brief Write to the specified GPIO pin
 * @param port Peripheral base address for the GPIO port of interest
 * @param pin  GPIO pin number
 * @param data Digital value to write to the specified GPIO pin
 */
void hal_gpio_Write(hal_gpio_Port_t *port, hal_gpio_Pin_t pin, bool data)
{

    if(data)
    {

        SET_BIT(port->BSRR, hal_gpio_PinMask(HAL_GPIO_BSRR_WIDTH, pin));

    }
    else
    {

        SET_BIT(port->BSRR, hal_gpio_PinMask(HAL_GPIO_BSRR_WIDTH, pin + 16));

    }

}

/**
 * @brief Write to the specified GPIO pin the logical not of its current state
 * @param port Peripheral base address for the GPIO port of interest
 * @param pin  GPIO pin number
 */
void hal_gpio_Toggle(hal_gpio_Port_t *port, hal_gpio_Pin_t pin)
{

    hal_gpio_Write(port, pin, !hal_gpio_Read(port, pin));

}


/*********************
 * Private Functions *
 *********************/

/**
 * @brief Return a shifted version of the provided config, shifted to fit into a GPIO configuration register
 * @param config       Config to be shifted
 * @param config_width Bit width of the config that's being shifted
 * @param pin          GPIO pin of interest
 */
static inline uint32_t hal_gpio_ShiftPinConfig(uint8_t config, uint8_t config_width, hal_gpio_Pin_t pin)
{

    return ((uint32_t)config) << (config_width * pin);

}

/**
 * @brief Return a mask for a pin config of a specified width
 * @param config_width Bit width of the config
 * @param pin          GPIO pin of interest
 */
static inline uint32_t hal_gpio_PinMask(uint8_t config_width, hal_gpio_Pin_t pin)
{

    return ((1 << config_width) - 1) << (config_width * pin);

}

