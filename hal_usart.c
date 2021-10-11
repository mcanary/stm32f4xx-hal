

/**
 * @file    hal_usart.c
 * @brief   STM32F4 USART peripheral HAL implementation source file
 */


#include "hal_usart.h"


/********************
 * Public Functions *
 ********************/

/**
 * @brief Configure the specified USART port
 * @param usart_port Peripheral base address for the USART port of interest
 * @param config     Pointer to a struct containing the USART port configuration parameters
 */
void hal_usart_Init(hal_usart_Port_t *usart_port, hal_usart_Config_s *config)
{

    MODIFY_REG(usart_port->BRR, HAL_USART_BRR_MASK, \
               (config->BaudMantissa << HAL_USART_BRR_DIV_MANTISSA_POSITION) | \
               (config->BaudFraction << HAL_USART_BRR_DIV_FRACTION_POSITION));

    MODIFY_REG(usart_port->CR1, HAL_USART_CR1_MASK, \
               (config->TransmitEnable << HAL_USART_CR1_TE_POSITION) | \
               (config->ReceiveEnable << HAL_USART_CR1_RE_POSITION) | \
               (config->ParityControlEnable << HAL_USART_CR1_PCE_POSITION) | \
               (config->ParitySelection << HAL_USART_CR1_PS_POSITION) | \
               (config->WordLength << HAL_USART_CR1_M_POSITION) | \
               (config->USART_Enable << HAL_USART_CR1_UE_POSITION) | \
               (config->OversamplingMode << HAL_USART_CR1_OVER8_POSITION));

    MODIFY_REG(usart_port->CR2, HAL_USART_CR2_STOP_MASK, (config->StopBits << HAL_USART_CR2_STOP_POSITION));

}

/**
 * @brief Transmit data over the USART port specified in a blocking mode
 * @param usart_port Peripheral base address for the USART port of interest
 * @param data       Pointer to an array of characters to transmit
 * @param length     Number of bytes to transmit
 */
void hal_usart_TransmitBlocking(hal_usart_Port_t *usart_port, char *data, uint32_t length)
{

    for(uint32_t data_index = 0; data_index < length; data_index++)
    {

        while(!READ_BIT(usart_port->SR, HAL_USART_SR_TXE_MASK));
        WRITE_REG(usart_port->DR, data[data_index]);

    }

}

