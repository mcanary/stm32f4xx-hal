

/**
 * @file    hal_flash.h
 * @brief   STM32F4xx flash peripheral HAL implementation header file
 */


#ifndef __HAL_FLASH
#define __HAL_FLASH


/************
 * Includes *
 ***********/

#include "stm32f4xx.h"


/***********
 * Defines *
 ***********/

/* FLASH_ACR Register Mask/Position */
#define HAL_FLASH_ACR_LATENCY_MASK     FLASH_ACR_LATENCY_Msk  // Flash latency mask
#define HAL_FLASH_ACR_LATENCY_POSITION FLASH_ACR_LATENCY_Pos  // Flash latency position

/* FLASH_ACR_LATENCY Enumerations */
#define HAL_FLASH_ACR_LATENCY_0_WS  (0)   // Flash latency = 0 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_1_WS  (1)   // Flash latency = 1 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_2_WS  (2)   // Flash latency = 2 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_3_WS  (3)   // Flash latency = 3 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_4_WS  (4)   // Flash latency = 4 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_5_WS  (5)   // Flash latency = 5 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_6_WS  (6)   // Flash latency = 6 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_7_WS  (7)   // Flash latency = 7 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_8_WS  (8)   // Flash latency = 8 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_9_WS  (9)   // Flash latency = 9 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_10_WS (10)  // Flash latency = 10 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_11_WS (11)  // Flash latency = 11 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_12_WS (12)  // Flash latency = 12 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_13_WS (13)  // Flash latency = 13 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_14_WS (14)  // Flash latency = 14 sysclk cycles
#define HAL_FLASH_ACR_LATENCY_15_WS (15)  // Flash latency = 15 sysclk cycles


/***************************
 * Public Type Definitions *
 ***************************/

typedef uint8_t hal_flash_Latency_t;


/******************************
 * Public Function Prototypes *
 ******************************/

void hal_flash_SetLatency(hal_flash_Latency_t latency);


#endif /* __HAL_FLASH */

