

/**
 * @file    hal_rcc.c
 * @brief   STM32F4 RCC peripheral HAL implementation source file
 */


/************
/* Includes *
 ***********/

#include "hal_rcc.h"


/********************
/* Public Functions *
 ********************/

/**
 * @brief Configure the main PLL
 * @param pll_config Pointer to a struct containing pll configuration parameters
 */
void hal_rcc_ConfigurePLL(hal_rcc_PLLConfigStruct *pll_config)
{

    MODIFY_REG(RCC->PLLCFGR, HAL_RCC_PLLCFG_MASK, \
        ((pll_config->PLL_M << HAL_RCC_PLLCFGR_PLLM_POSITION) | \
         (pll_config->PLL_N << HAL_RCC_PLLCFGR_PLLN_POSITION) | \
         (pll_config->PLL_P << HAL_RCC_PLLCFGR_PLLP_POSITION) | \
         (pll_config->PLL_Source << HAL_RCC_PLLCFGR_PLLSRC_POSITION) | \
         (pll_config->PLL_Q << HAL_RCC_PLLCFGR_PLLQ_POSITION)));

}

/**
 * @brief Configure system and peripheral clocks
 * @param clock_config Pointer to a struct containing clock configuration parameters
 */
void hal_rcc_ConfigureClocks(hal_rcc_ClockConfigStruct *clock_config)
{

    MODIFY_REG(RCC->CFGR, HAL_RCC_CFGR_MASK, \
        ((clock_config->AHB_Prescaler << HAL_RCC_CFGR_HPRE_POSITION) | \
         (clock_config->APB1_Prescaler << HAL_RCC_CFGR_PPRE1_POSITION) | \
         (clock_config->APB2_Prescaler << HAL_RCC_CFGR_PPRE2_POSITION) | \
         (clock_config->RTC_Prescaler << HAL_RCC_CFGR_RTCPRE_POSITION) | \
         (clock_config->MCO1_Source << HAL_RCC_CFGR_MCO1_POSITION) | \
         (clock_config->MCO2_Source << HAL_RCC_CFGR_MCO2_POSITION) | \
         (clock_config->MCO1_Prescaler << HAL_RCC_CFGR_MCO1PRE_POSITION) | \
         (clock_config->MCO2_Prescaler << HAL_RCC_CFGR_MCO2PRE_POSITION) | \
         (clock_config->I2S_Source << HAL_RCC_CFGR_I2SSRC_POSITION)));

}

/**
 * @brief Configure the I2S PLL
 * @param i2s_pll_config Pointer to a struct containing I2S PLL configuration parameters
 */
void hal_rcc_ConfigureI2SPLL(hal_rcc_I2SPLLConfigStruct *i2s_pll_config)
{

    MODIFY_REG(RCC->PLLI2SCFGR, HAL_RCC_PLLI2SCFGR_MASK, \
        ((i2s_pll_config->PLLI2S_M << HAL_RCC_PLLI2SCFGR_PLLI2SM_POSITION) | \
         (i2s_pll_config->PLLI2S_N << HAL_RCC_PLLI2SCFGR_PLLI2SN_POSITION) | \
         (i2s_pll_config->PLLI2S_R << HAL_RCC_PLLI2SCFGR_PLLI2SR_POSITION)));

}

/**
 * @brief Set the system clock source
 * @param source System clock source setting
 */
void hal_rcc_SetSystemClockSource(hal_rcc_SystemClockSource source)
{

    MODIFY_REG(RCC->CFGR, HAL_RCC_CFGR_SW_MASK, source);

}

/**
 * @brief  Check if the HSI oscillator is stable/ready
 * @return True if HSI is stable/ready
 */
bool hal_rcc_HSIReady(void)
{

     return READ_BIT(RCC->CR, HAL_RCC_CR_HSIRDY_MASK);

}

/**
 * @brief  Check if the HSE oscillator is stable/ready
 * @return True if HSE is stable/ready
 */
bool hal_rcc_HSEReady(void)
{

    return READ_BIT(RCC->CR, HAL_RCC_CR_HSERDY_MASK);

}

/**
 * @brief  Check if the main PLL is stable/ready
 * @return True if the main PLL is stable/ready
 */
bool hal_rcc_PLLReady(void)
{

    return READ_BIT(RCC->CR, HAL_RCC_CR_PLLRDY_MASK);

}

/**
 * @brief  Check if the I2S PLL is stable/ready
 * @return True if the I2S PLL is stable/ready
 */
bool hal_rcc_PLLI2SReady(void)
{

    return READ_BIT(RCC->CR, HAL_RCC_CR_PLLI2SRDY_MASK);

}

/**
 * @brief Enable the HSI oscillator
 */
void hal_rcc_EnableHSI(void)
{

    SET_BIT(rcc->CR, HAL_RCC_CR_HSION_MASK);

}

/**
 * @brief Disable the HSI oscillator
 */
void hal_rcc_DisableHSI(void)
{

    CLEAR_BIT(rcc->CR, HAL_RCC_CR_HSION_MASK);

}

/**
 * @brief Enable the HSE oscillator
 */
void hal_rcc_EnableHSE(void)
{

    SET_BIT(RCC->CR, HAL_RCC_CR_HSEON_MASK);

}

/**
 * @brief Disable the HSE oscillator
 */
void hal_rcc_DisableHSE(void)
{

    CLEAR_BIT(RCC->CR, HAL_RCC_CR_HSEON_MASK);

}

/**
 * @brief Enable the main PLL
 */
void hal_rcc_EnablePLL(void)
{

    SET_BIT(RCC->CR, HAL_RCC_CR_PLLON_MASK);

}

/**
 * @brief Disable the main PLL
 */
void hal_rcc_DisablePLL(void)
{

    CLEAR_BIT(RCC->CR, HAL_RCC_CR_PLLON_MASK);

}

/**
 * @brief Enable the I2S PLL
 */
void hal_rcc_EnablePLLI2S(void)
{

    SET_BIT(RCC->CR, HAL_RCC_CR_PLLI2SON_MASK);

}

/**
 * @brief Disable the I2S PLL
 */
void hal_rcc_DisablePLLI2S(void)
{

    CLEAR_BIT(RCC->CR, HAL_RCC_CR_PLLI2SON_MASK);

}

/**
 * @brief Enable the peripheral clock for the specified GPIO port
 * @param gpio_port GPIO port for which the peripheral clock should be enabled
 */
void hal_rcc_EnableGPIOClock(hal_gpio_Port gpio_port)
{

    switch(gpio_port)
    {

        case GPIOA:

            SET_BIT(RCC->AHB1ENR, HAL_RCC_AHB1ENR_GPIOAENR_MASK);
            break;

        case GPIOB:

            SET_BIT(RCC->AHB1ENR, HAL_RCC_AHB1ENR_GPIOBENR_MASK);
            break;

        case GPIOC:

            SET_BIT(RCC->AHB1ENR, HAL_RCC_AHB1ENR_GPIOCENR_MASK);
            break;

        case GPIOD:

            SET_BIT(RCC->AHB1ENR, HAL_RCC_AHB1ENR_GPIODENR_MASK);
            break;

        case GPIOE:

            SET_BIT(RCC->AHB1ENR, HAL_RCC_AHB1ENR_GPIOEENR_MASK);
            break;

        case GPIOH:

            SET_BIT(RCC->AHB1ENR, HAL_RCC_AHB1ENR_GPIOHENR_MASK);
            break;

        default:

            break;

    }

}

/**
 * @brief Enable the peripheral clock for the specified USART port
 * @param usart_port USART port for which the peripheral clock should be enabled
 */
void hal_rcc_EnableUSARTClock(hal_usart_Port usart_port)
{

    switch(usart_port)
    {

        case USART1:

            SET_BIT(RCC->APB2ENR, HAL_RCC_APB2ENR_USART1EN_MASK);
            break;

        case USART6:

            SET_BIT(RCC->APB2ENR, HAL_RCC_APB2ENR_USART6EN_MASK);
            break;

        default:

            break;

    }

}

