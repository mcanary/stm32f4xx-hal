

/**
 * @file    hal_i2c.h
 * @brief   STM32F4 I2C peripheral HAL implementation header file
 */


#ifndef __HAL_I2C
#define __HAL_I2C


/************
/* Includes *
 ************/

#include "stm32f4xx.h"


/***********
/* Defines *
 ***********/

/* I2C_CR1 Register Mask/Position */
#define HAL_I2C_CR1_PE_MASK    I2C_CR1_PE_Msk     // Peripheral enable mask
#define HAL_I2C_CR1_START_MASK I2C_CR1_START_Msk  // Start bit send mask
#define HAL_I2C_CR1_STOP_MASK  I2C_STOP_STOP_Msk  // Stop bit send mask
#define HAL_I2C_CR1_SWRST_MASK I2C_CR1_SWRST_Msk  // Software reset mask

#define HAL_I2C_CR1_PE_POSITION    I2C_CR1_PE_Pos     // Peripheral enable position
#define HAL_I2C_CR1_START_POSITION I2C_CR1_START_Pos  // Start bit send position
#define HAL_I2C_CR1_STOP_POSITION  I2C_STOP_STOP_Pos  // Stop bit send position
#define HAL_I2C_CR1_SWRST_POSITION I2C_CR1_SWRST_Pos  // Software reset position

/* I2C_CR2 Register Mask/Position */
#define HAL_I2C_CR2_ITEVTEN_MASK I2C_CR2_ITEVTEN_Msk  // Event interrupt enable mask
#define HAL_I2C_CR2_FREQ_MASK    I2C_CR2_FREQ_Msk     // Peripheral clock frequency mask

#define HAL_I2C_CR2_ITEVTEN_POSITION I2C_CR2_ITEVTEN_Pos  // Event interrupt enable position
#define HAL_I2C_CR2_FREQ_POSITION    I2C_CR2_FREQ_Pos     // Peripheral clock frequency position

/* I2C_SR1 Register Mask/Position */
#define HAL_I2C_SR1_SB_MASK   I2C_SR1_SB_Msk    // Start condition generated mask
#define HAL_I2C_SR1_ADDR_MASK I2C_SR1_ADDR_Msk  // Address sent mask
#define HAL_I2C_SR1_BTF_MASK  I2C_SR1_BTF_Msk   // Byte transfer finished mask
#define HAL_I2C_SR1_RXNE_MASK I2C_SR1_RXNE_Msk  // Receive register not empty mask
#define HAL_I2C_SR1_TXE_MASK  I2C_SR1_TXE_Msk   // Transmit register empty mask

#define HAL_I2C_SR1_SB_POSITION   I2C_SR1_SB_Pos    // Start condition generated position
#define HAL_I2C_SR1_ADDR_POSITION I2C_SR1_ADDR_Pos  // Address sent position
#define HAL_I2C_SR1_BTF_POSITION  I2C_SR1_BTF_Pos   // Byte transfer finished position
#define HAL_I2C_SR1_RXNE_POSITION I2C_SR1_RXNE_Pos  // Receive register not empty position
#define HAL_I2C_SR1_TXE_POSITION  I2C_SR1_TXE_Pos   // Transmit register empty position

/* I2C_SR2 Register Mask/Position */
#define HAL_I2C_SR2_BUSY_MASK     I2C_SR1_BUSY_Msk    // Bus busy mask
#define HAL_I2C_SR2_BUSY_POSITION I2C_SR1_BUSY_Pos    // Bus busy position

/* I2C_CCR Register Mask/Position */
#define HAL_I2C_CCR_CCR_MASK I2C_CCR_CCR_Msk  // Clock control register mask
#define HAL_I2C_CCR_FS_MASK  I2C_CCR_FS_Msk   // Mode selection mask

#define HAL_I2C_CCR_CCR_POSITION I2C_CCR_CCR_Pos  // Clock control register position
#define HAL_I2C_CCR_FS_POSITION  I2C_CCR_FS_Pos   // Mode selection position

/* I2C_CCR Config Mask */
#define HAL_I2C_CCR_MASK (I2C_CCR_FS_Msk | I2C_CCR_CCR_Msk)

/* I2C Read/Write Bit Enumerations */
#define HAL_I2C_RWBIT_WRITE (0x0)  // Write address suffix
#define HAL_I2C_RWBIT_READ  (0x1)  // Read address suffix

/* Defaults */
#define HAL_I2C_NUM_PERIPHERALS (3)  // Total number of I2C peripheral instances available


/***************************
/* Public Type Definitions *
 ***************************/

typedef I2C_TypeDef HAL_I2C_Port;

typedef struct HAL_I2C_ConfigStruct
{
    uint32_t APB_ClockFrequency;  // APB clock frequency
    bool FastMode;                // Fast mode enable
    bool I2C_Enable;              // I2C peripheral enable
}HAL_I2C_ConfigStruct;


/******************************
/* Public Function Prototypes *
 ******************************/

void HAL_I2C_Init(HAL_I2C_Port *i2c_port, HAL_I2C_ConfigStruct *config);
bool HAL_I2C_IsBusy(HAL_I2C_Port *i2c_port);
void HAL_I2C_WriteDataBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length);
void HAL_I2C_ReadDataBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length);
void HAL_I2C_WriteDataNonBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length);
void HAL_I2C_ReadDataNonBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length);


#endif /* __HAL_I2C */

