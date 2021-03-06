

/**
 * @file    hal_rcc.h
 * @brief   STM32F4 RCC peripheral HAL implementation header file
 */


#ifndef __HAL_RCC
#define __HAL_RCC


/************
 * Includes *
 ************/

#include "stm32f4xx.h"

#include "hal_gpio.h"
#include "hal_i2c.h"
#include "hal_i2s.h"
#include "hal_usart.h"


/***********
 * Defines *
 ***********/

/* RCC_CFGR Register Mask/Position */
#define HAL_RCC_CR_HSION_MASK     RCC_CR_HSION_Msk      // HSI enable mask
#define HAL_RCC_CR_HSIRDY_MASK    RCC_CR_HSIRDY_Msk     // HSI ready mask
#define HAL_RCC_CR_HSEON_MASK     RCC_CR_HSEON_Msk      // HSE enable mask
#define HAL_RCC_CR_HSERDY_MASK    RCC_CR_HSERDY_Msk     // HSE ready mask
#define HAL_RCC_CR_HSEBYP_MASK    RCC_CR_HSEBYP_Msk     // HSE bypass mask
#define HAL_RCC_CR_PLLON_MASK     RCC_CR_PLLON_Msk      // Main PLL enable mask
#define HAL_RCC_CR_PLLRDY_MASK    RCC_CR_PLLRDY_Msk     // Main PLL ready mask
#define HAL_RCC_CR_PLLI2SON_MASK  RCC_CR_PLLI2SON_Msk   // I2S PLL enable mask
#define HAL_RCC_CR_PLLI2SRDY_MASK RCC_CR_PLLI2SRDY_Msk  // I2S PLL ready mask

#define HAL_RCC_CR_HSION_POSITION     RCC_CR_HSION_Pos      // HSI enable position
#define HAL_RCC_CR_HSIRDY_POSITION    RCC_CR_HSIRDY_Pos     // HSI ready position
#define HAL_RCC_CR_HSEON_POSITION     RCC_CR_HSION_Pos      // HSE enable position
#define HAL_RCC_CR_HSERDY_POSITION    RCC_CR_HSERDY_Pos     // HSE ready position
#define HAL_RCC_CR_HSEBYP_POSITION    RCC_CR_HSEBYP_Pos     // HSE bypass position
#define HAL_RCC_CR_PLLON_POSITION     RCC_CR_PLLON_Pos      // Main PLL enable position
#define HAL_RCC_CR_PLLRDY_POSITION    RCC_CR_PLLRDY_Pos     // Main PLL ready position
#define HAL_RCC_CR_PLLI2SON_POSITION  RCC_CR_PLLI2SON_Pos   // I2S PLL enable position
#define HAL_RCC_CR_PLLI2SRDY_POSITION RCC_CR_PLLI2SRDY_Pos  // I2S PLL ready position

/* RCC_PLLCFGR Register Mask/Position */
#define HAL_RCC_PLLCFGR_PLLM_MASK   RCC_PLLCFGR_PLLM_Msk    // Main PLL M mask
#define HAL_RCC_PLLCFGR_PLLN_MASK   RCC_PLLCFGR_PLLN_Msk    // Main PLL N mask
#define HAL_RCC_PLLCFGR_PLLP_MASK   RCC_PLLCFGR_PLLP_Msk    // Main PLL P mask
#define HAL_RCC_PLLCFGR_PLLSRC_MASK RCC_PLLCFGR_PLLSRC_Msk  // Main PLL source mask
#define HAL_RCC_PLLCFGR_PLLQ_MASK   RCC_PLLCFGR_PLLQ_Msk    // Main PLL M mask

#define HAL_RCC_PLLCFGR_PLLM_POSITION   RCC_PLLCFGR_PLLM_Pos    // Main PLL M position
#define HAL_RCC_PLLCFGR_PLLN_POSITION   RCC_PLLCFGR_PLLN_Pos    // Main PLL N position
#define HAL_RCC_PLLCFGR_PLLP_POSITION   RCC_PLLCFGR_PLLP_Pos    // Main PLL P position
#define HAL_RCC_PLLCFGR_PLLSRC_POSITION RCC_PLLCFGR_PLLSRC_Pos  // Main PLL source position
#define HAL_RCC_PLLCFGR_PLLQ_POSITION   RCC_PLLCFGR_PLLQ_Pos    // Main PLL M position

/* RCC_CFGR Register Mask/Position */
#define HAL_RCC_CFGR_SW_MASK      RCC_CFGR_SW_Msk       // System clock switch mask
#define HAL_RCC_CFGR_SWS_MASK     RCC_CFGR_SWS_Msk      // System clock source status mask
#define HAL_RCC_CFGR_HPRE_MASK    RCC_CFGR_HPRE_Msk     // AHB prescaler mask
#define HAL_RCC_CFGR_PPRE1_MASK   RCC_CFGR_PPRE1_Msk    // APB low-speed (APB1) prescaler mask
#define HAL_RCC_CFGR_PPRE2_MASK   RCC_CFGR_PPRE2_Msk    // APB high-speed (APB2) prescaler mask
#define HAL_RCC_CFGR_RTCPRE_MASK  RCC_CFGR_RTCPRE_Msk   // RTC prescaler mask
#define HAL_RCC_CFGR_MCO1_MASK    RCC_CFGR_MCO1_Msk     // MCO1 source mask
#define HAL_RCC_CFGR_I2SSRC_MASK  RCC_CFGR_I2SSRC_Msk   // I2S clock source mask
#define HAL_RCC_CFGR_MCO1PRE_MASK RCC_CFGR_MCO1PRE_Msk  // MCO1 prescaler mask
#define HAL_RCC_CFGR_MCO2PRE_MASK RCC_CFGR_MCO2PRE_Msk  // MCO2 prescaler mask
#define HAL_RCC_CFGR_MCO2_MASK    RCC_CFGR_MCO2_Msk     // MCO2 source mask

#define HAL_RCC_CFGR_SW_POSITION      RCC_CFGR_SW_Pos       // System clock switch position
#define HAL_RCC_CFGR_SWS_POSITION     RCC_CFGR_SWS_Pos      // System clock source status positon
#define HAL_RCC_CFGR_HPRE_POSITION    RCC_CFGR_HPRE_Pos     // AHB prescaler position
#define HAL_RCC_CFGR_PPRE1_POSITION   RCC_CFGR_PPRE1_Pos    // APB low-speed (APB1) prescaler position
#define HAL_RCC_CFGR_PPRE2_POSITION   RCC_CFGR_PPRE2_Pos    // APB high-speed (APB2) prescaler position
#define HAL_RCC_CFGR_RTCPRE_POSITION  RCC_CFGR_RTCPRE_Pos   // RTC prescaler position
#define HAL_RCC_CFGR_MCO1_POSITION    RCC_CFGR_MCO1_Pos     // MCO1 source position
#define HAL_RCC_CFGR_I2SSRC_POSITION  RCC_CFGR_I2SSRC_Pos   // I2S clock source position
#define HAL_RCC_CFGR_MCO1PRE_POSITION RCC_CFGR_MCO1PRE_Pos  // MCO1 prescaler position
#define HAL_RCC_CFGR_MCO2PRE_POSITION RCC_CFGR_MCO2PRE_Pos  // MCO2 prescaler position
#define HAL_RCC_CFGR_MCO2_POSITION    RCC_CFGR_MCO2_Pos     // MCO2 source position

/* RCC_PLLI2SCFGR Register Mask/Position */
#define HAL_RCC_PLLI2SCFGR_PLLI2SM_MASK RCC_PLLI2SCFGR_PLLI2SM_Msk  // I2S PLL M mask
#define HAL_RCC_PLLI2SCFGR_PLLI2SN_MASK RCC_PLLI2SCFGR_PLLI2SN_Msk  // I2S PLL N mask
#define HAL_RCC_PLLI2SCFGR_PLLI2SR_MASK RCC_PLLI2SCFGR_PLLI2SR_Msk  // I2S PLL R mask

#define HAL_RCC_PLLI2SCFGR_PLLI2SM_POSITION RCC_PLLI2SCFGR_PLLI2SM_Pos  // I2S PLL M position
#define HAL_RCC_PLLI2SCFGR_PLLI2SN_POSITION RCC_PLLI2SCFGR_PLLI2SN_Pos  // I2S PLL N position
#define HAL_RCC_PLLI2SCFGR_PLLI2SR_POSITION RCC_PLLI2SCFGR_PLLI2SR_Pos  // I2S PLL R position

/* RCC_AHB1ENR Register Mask/Position */
#define HAL_RCC_AHB1ENR_GPIOAEN_MASK RCC_AHB1ENR_GPIOAEN_Msk  // GPIOA peripheral clock enable mask
#define HAL_RCC_AHB1ENR_GPIOBEN_MASK RCC_AHB1ENR_GPIOBEN_Msk  // GPIOB peripheral clock enable mask
#define HAL_RCC_AHB1ENR_GPIOCEN_MASK RCC_AHB1ENR_GPIOCEN_Msk  // GPIOC peripheral clock enable mask
#define HAL_RCC_AHB1ENR_GPIODEN_MASK RCC_AHB1ENR_GPIODEN_Msk  // GPIOD peripheral clock enable mask
#define HAL_RCC_AHB1ENR_GPIOEEN_MASK RCC_AHB1ENR_GPIOEEN_Msk  // GPIOE peripheral clock enable mask
#define HAL_RCC_AHB1ENR_GPIOHEN_MASK RCC_AHB1ENR_GPIOHEN_Msk  // GPIOH peripheral clock enable mask

#define HAL_RCC_AHB1ENR_GPIOAEN_POSITION RCC_AHB1ENR_GPIOAEN_Pos  // GPIOA peripheral clock enable position
#define HAL_RCC_AHB1ENR_GPIOBEN_POSITION RCC_AHB1ENR_GPIOBEN_Pos  // GPIOB peripheral clock enable position
#define HAL_RCC_AHB1ENR_GPIOCEN_POSITION RCC_AHB1ENR_GPIOCEN_Pos  // GPIOC peripheral clock enable position
#define HAL_RCC_AHB1ENR_GPIODEN_POSITION RCC_AHB1ENR_GPIODEN_Pos  // GPIOD peripheral clock enable position
#define HAL_RCC_AHB1ENR_GPIOEEN_POSITION RCC_AHB1ENR_GPIOEEN_Pos  // GPIOE peripheral clock enable position
#define HAL_RCC_AHB1ENR_GPIOHEN_POSITION RCC_AHB1ENR_GPIOHEN_Pos  // GPIOH peripheral clock enable position

/* RCC_APB1ENR Register Mask/Position */
#define HAL_RCC_APB1ENR_I2C1EN_MASK RCC_APB1ENR_I2C1EN_Msk  // I2C1 peripheral clock enable mask
#define HAL_RCC_APB1ENR_I2C2EN_MASK RCC_APB1ENR_I2C2EN_Msk  // I2C2 peripheral clock enable mask
#define HAL_RCC_APB1ENR_I2C3EN_MASK RCC_APB1ENR_I2C3EN_Msk  // I2C3 peripheral clock enable mask
#define HAL_RCC_APB1ENR_SPI2EN_MASK RCC_APB1ENR_SPI2EN_Msk  // SPI2 peripheral clock enable mask
#define HAL_RCC_APB1ENR_SPI3EN_MASK RCC_APB1ENR_SPI3EN_Msk  // SPI3 peripheral clock enable mask

#define HAL_RCC_APB1ENR_I2C1EN_POSITION RCC_APB1ENR_I2C1EN_Pos  // I2C1 peripheral clock enable position
#define HAL_RCC_APB1ENR_I2C2EN_POSITION RCC_APB1ENR_I2C2EN_Pos  // I2C2 peripheral clock enable position
#define HAL_RCC_APB1ENR_I2C3EN_POSITION RCC_APB1ENR_I2C3EN_Pos  // I2C3 peripheral clock enable position
#define HAL_RCC_APB1ENR_SPI2EN_POSITION RCC_APB1ENR_SPI2EN_Pos  // SPI2 peripheral clock enable position
#define HAL_RCC_APB1ENR_SPI3EN_POSITION RCC_APB1ENR_SPI3EN_Pos  // SPI3 peripheral clock enable position

/* RCC_APB2ENR Register Mask/Position */
#define HAL_RCC_APB2ENR_USART1EN_MASK RCC_APB2ENR_USART1EN_Msk  // USART1 peripheral clock enable mask
#define HAL_RCC_APB2ENR_USART6EN_MASK RCC_APB2ENR_USART6EN_Msk  // USART6 peripheral clock enable mask

#define HAL_RCC_APB2ENR_USART1EN_POSITION RCC_APB2ENR_USART1EN_Pos  // USART1 peripheral clock enable position
#define HAL_RCC_APB2ENR_USART6EN_POSITION RCC_APB2ENR_USART6EN_Pos  // USART6 peripheral clock enable position

/* RCC_CFGR_SW (System Clock Switch) Enumerations */
#define HAL_RCC_CFGR_SW_HSI (0x0)  // High-speed internal oscillator
#define HAL_RCC_CFGR_SW_HSE (0x1)  // High-speed external oscillator
#define HAL_RCC_CFGR_SW_PLL (0x2)  // Main PLL

/* RCC_CFGR_HPRE (AHB Clock Prescaler) Enumerations */
#define HAL_RCC_CFGR_AHBPRE_1   (0x0)  // AHBPRE = 1
#define HAL_RCC_CFGR_AHBPRE_2   (0x8)  // AHBPRE = 2
#define HAL_RCC_CFGR_AHBPRE_4   (0x9)  // AHBPRE = 4
#define HAL_RCC_CFGR_AHBPRE_8   (0xa)  // AHBPRE = 8
#define HAL_RCC_CFGR_AHBPRE_16  (0xb)  // AHBPRE = 16
#define HAL_RCC_CFGR_AHBPRE_64  (0xc)  // AHBPRE = 64
#define HAL_RCC_CFGR_AHBPRE_128 (0xd)  // AHBPRE = 128
#define HAL_RCC_CFGR_AHBPRE_256 (0xe)  // AHBPRE = 256
#define HAL_RCC_CFGR_AHBPRE_512 (0xf)  // AHBPRE = 512

/* RCC_CFGR_PPRE[1, 2] (APB1/2 Clock Prescaler) Enumerations */
#define HAL_RCC_CFGR_APBPRE_1  (0x0)  // APBPRE = 1
#define HAL_RCC_CFGR_APBPRE_2  (0x4)  // APBPRE = 2
#define HAL_RCC_CFGR_APBPRE_4  (0x5)  // APBPRE = 4
#define HAL_RCC_CFGR_APBPRE_8  (0x6)  // APBPRE = 8
#define HAL_RCC_CFGR_APBPRE_16 (0x7)  // APBPRE = 16

/* RCC_CFGR_MCO1 (MCO1 Source) Enumerations */
#define HAL_RCC_CFGR_MCO1_HSI (0x0)  // High-speed internal oscillator
#define HAL_RCC_CFGR_MCO1_LSE (0x1)  // Low-speed external oscillator
#define HAL_RCC_CFGR_MCO1_HSE (0x2)  // High-speed external oscillator
#define HAL_RCC_CFGR_MCO1_PLL (0x3)  // Main PLL

/* RCC_CFGR_MCO2 (MCO2 Source) Enumerations */
#define HAL_RCC_CFGR_MCO2_SYSCLK (0x0)  // System clock
#define HAL_RCC_CFGR_MCO2_PLLI2S (0x1)  // I2S PLL
#define HAL_RCC_CFGR_MCO2_HSE    (0x2)  // High-speed external oscillator
#define HAL_RCC_CFGR_MCO2_PLL    (0x3)  // Main PLL

/* RCC_CFGR_I2SSRC (I2S PLL Clock Source) Enumerations */
#define HAL_RCC_CFGR_I2SSOURCE_PLLI2S  (0x0)  // I2S PLL
#define HAL_RCC_CFGR_I2SSOURCE_I2SCKIN (0x1)  // External I2S clock input

/* RCC_CFGR_MCO[1, 2]PRE (MCO Prescaler) Enumerations */
#define HAL_RCC_CFGR_MCOPRE_1 (0x0)  // MCOPRE = 1
#define HAL_RCC_CFGR_MCOPRE_2 (0x4)  // MCOPRE = 2
#define HAL_RCC_CFGR_MCOPRE_3 (0x5)  // MCOPRE = 3
#define HAL_RCC_CFGR_MCOPRE_4 (0x6)  // MCOPRE = 4
#define HAL_RCC_CFGR_MCOPRE_5 (0x7)  // MCOPRE = 5

/* RCC_PLLCFGR_PLLP (Main PLL P) Enumerations */
#define HAL_RCC_PLLCFGR_PLLP_2 (0x0)  // PLLP = 2
#define HAL_RCC_PLLCFGR_PLLP_4 (0x1)  // PLLP = 4
#define HAL_RCC_PLLCFGR_PLLP_6 (0x2)  // PLLP = 6
#define HAL_RCC_PLLCFGR_PLLP_8 (0x3)  // PLLP = 8

/* RCC_PLLCFGR_PLLSRC (Main PLL Clock Source) Enumerations */
#define HAL_RCC_PLLCFGR_PLLSRC_HSI (0x0)  // High-speed internal oscillator
#define HAL_RCC_PLLCFGR_PLLSRC_HSE (0x1)  // High-speed external oscillator

/* RCC_PLLCFG Config Mask */
#define HAL_RCC_PLLCFG_MASK (HAL_RCC_PLLCFGR_PLLM_MASK | \
                             HAL_RCC_PLLCFGR_PLLN_MASK | \
                             HAL_RCC_PLLCFGR_PLLP_MASK | \
                             HAL_RCC_PLLCFGR_PLLSRC_MASK | \
                             HAL_RCC_PLLCFGR_PLLQ_MASK)

/* RCC_CFGR Config Mask */
#define HAL_RCC_CFGR_MASK (HAL_RCC_CFGR_HPRE_MASK | \
                           HAL_RCC_CFGR_PPRE1_MASK | \
                           HAL_RCC_CFGR_PPRE2_MASK | \
                           HAL_RCC_CFGR_RTCPRE_MASK | \
                           HAL_RCC_CFGR_MCO1_MASK | \
                           HAL_RCC_CFGR_I2SSRC_MASK | \
                           HAL_RCC_CFGR_MCO1PRE_MASK | \
                           HAL_RCC_CFGR_MCO2PRE_MASK | \
                           HAL_RCC_CFGR_MCO2_MASK)

/* RCC_PLLI2SCFGR Config Mask */
#define HAL_RCC_PLLI2SCFGR_MASK (HAL_RCC_PLLI2SCFGR_PLLI2SM_MASK | \
                                 HAL_RCC_PLLI2SCFGR_PLLI2SN_MASK | \
                                 HAL_RCC_PLLI2SCFGR_PLLI2SR_MASK)


/******************** 
 * Type Definitions *
 ********************/

typedef uint8_t hal_rcc_SystemClockSource_t;
typedef uint8_t hal_rcc_PLLSource_t;
typedef uint8_t hal_rcc_PLLP_t;
typedef uint8_t hal_rcc_MCO1Source_t;
typedef uint8_t hal_rcc_MCO2Source_t;
typedef uint8_t hal_rcc_PLLI2SSource_t;

typedef struct hal_rcc_PLLConfig_s
{
    uint8_t             PLL_M;       // Main PLL M
    uint16_t            PLL_N;       // Main PLL N
    hal_rcc_PLLP_t      PLL_P;       // Main PLL P
    hal_rcc_PLLSource_t PLL_Source;  // Main PLL source
    uint8_t             PLL_Q;       // Main PLL Q
}hal_rcc_PLLConfig_s;

typedef struct hal_rcc_ClockConfig_s
{
    uint8_t                AHB_Prescaler;   // Prescaler for AHB peripheral clocks
    uint8_t                APB1_Prescaler;  // Prescaler for APB1 peripheral clocks
    uint8_t                APB2_Prescaler;  // Prescaler for APB2 peripheral clocks
    uint8_t                RTC_Prescaler;   // RTC prescaler
    hal_rcc_MCO1Source_t   MCO1_Source;     // Microcontroller clock output 1 source
    hal_rcc_MCO2Source_t   MCO2_Source;     // Microcontroller clock output 2 source
    uint8_t                MCO1_Prescaler;  // Microcontroller clock output 2 prescaler
    uint8_t                MCO2_Prescaler;  // Microcontroller clock output 2 prescaler
    hal_rcc_PLLI2SSource_t I2S_Source;      // I2S PLL source
}hal_rcc_ClockConfig_s;

typedef struct hal_rcc_I2SPLLConfig_s
{
    uint8_t  PLLI2S_M;  // I2S PLL M
    uint16_t PLLI2S_N;  // I2S PLL N
    uint8_t  PLLI2S_R;  // I2S PLL R
}hal_rcc_I2SPLLConfig_s;


/******************************
 * Public Function Prototypes *
 ******************************/

void hal_rcc_ConfigurePLL(hal_rcc_PLLConfig_s *pll_config);
void hal_rcc_ConfigureClocks(hal_rcc_ClockConfig_s *clock_config);
void hal_rcc_ConfigureI2SPLL(hal_rcc_I2SPLLConfig_s *i2s_pll_config);
void hal_rcc_SetSystemClockSource(hal_rcc_SystemClockSource_t source);
bool hal_rcc_HSIReady(void);
bool hal_rcc_HSEReady(void);
bool hal_rcc_PLLReady(void);
bool hal_rcc_PLLI2SReady(void);
void hal_rcc_EnableHSI(void);
void hal_rcc_DisableHSI(void);
void hal_rcc_EnableHSE(void);
void hal_rcc_DisableHSE(void);
void hal_rcc_EnablePLL(void);
void hal_rcc_DisablePLL(void);
void hal_rcc_EnablePLLI2S(void);
void hal_rcc_DisablePLLI2S(void);
void hal_rcc_EnableGPIOClock(hal_gpio_Port_t *gpio_port);
void hal_rcc_EnableI2CClock(hal_i2c_Port_t *i2c_port);
void hal_rcc_EnableI2SClock(hal_i2s_Port_t *i2s_port);
void hal_rcc_EnableUSARTClock(hal_usart_Port_t *usart_port);


#endif /* __HAL_RCC */

