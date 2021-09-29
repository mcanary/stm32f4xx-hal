

/**
 * @file   hal_flash.c
 * @brief  STM32F4xx flash peripheral HAL implementation source file
 */


/************
/* Includes *
 ***********/

#include "hal_flash.h"


/********************
/* Public Functions *
 ********************/

/**
 * @brief Set flash access latency
 * @param latency Flash access latency in sysclk cycles
 */
void hal_flash_SetLatency(hal_flash_Latency latency)
{

    MODIFY_REG(FLASH->ACR, HAL_FLASH_ACR_LATENCY_MASK, latency);

}

