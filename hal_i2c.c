

/**
 * @file    hal_i2c.c
 * @brief   STM32F4 I2C peripheral HAL implementation source file
 */


/************
 * Includes *
 ************/

#include "hal_i2c.h"


/*******************************
 * Private Function Prototypes *
 *******************************/

static void hal_i2c_SendStart(hal_i2c_Port_t *i2c_port);
static void hal_i2c_SendAddress(hal_i2c_Port_t *i2c_port, uint8_t address, bool read_enable);
static void hal_i2c_SendByte(hal_i2c_Port_t *i2c_port, uint8_t data);
static uint8_t hal_i2c_ReceiveByte(hal_i2c_Port_t *i2c_port);
static void hal_i2c_SendStop(hal_i2c_Port_t *i2c_port);


/********************
 * Public Functions *
 ********************/

/**
 * @brief Configure the specified I2C port
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @param config   Pointer to a struct containing the I2C port configuration parameters
 */
void hal_i2c_Init(hal_i2c_Port_t *i2c_port, hal_i2c_Config_s *config)
{

    SET_BIT(i2c_port->CR1, HAL_I2C_CR1_SWRST_MASK);
    CLEAR_BIT(i2c_port->CR1, HAL_I2C_CR1_SWRST_MASK);
    CLEAR_BIT(i2c_port->CR1, HAL_I2C_CR1_PE_MASK);

    if(config->FastMode)
    {

        MODIFY_REG(i2c_port->CCR, HAL_I2C_CCR_MASK, (uint32_t)(HAL_I2C_CCR_FS_MASK | (config->APB_ClockFrequency / 800000)));
        WRITE_REG(i2c_port->TRISE, (uint32_t)(0.3 * (config->APB_ClockFrequency / 1000000)));

    }
    else
    {

        MODIFY_REG(i2c_port->CCR, HAL_I2C_CCR_CCR_MASK, (uint32_t)(config->APB_ClockFrequency / 100000));
        CLEAR_BIT(i2c_port->CCR, HAL_I2C_CCR_FS_MASK);
        WRITE_REG(i2c_port->TRISE, (uint32_t)(config->APB_ClockFrequency / 1000000));

    }

    MODIFY_REG(i2c_port->CR2, HAL_I2C_CR2_FREQ_MASK, (uint32_t)(config->APB_ClockFrequency / 1000000));
    SET_BIT(i2c_port->CR1, HAL_I2C_CR1_PE_MASK);

}

/**
 * @brief Check if the I2C bus is busy
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @return True if the peripheral is busy
 */
bool hal_i2c_IsBusy(hal_i2c_Port_t *i2c_port)
{

    return (bool)READ_BIT(i2c_port->SR2, HAL_I2C_SR2_BUSY_MASK);

}

/**
 * @brief Write data to the I2C bus in a blocking manner
 * @param i2c_port       Peripheral base address for the I2C port of interest
 * @param dev_address    Device address to write to
 * @param reg_address    Register address to write to
 * @param address_length Length of the register address
 * @param data           Data to write to the specified register address
 * @param data_length    Length of the data to be written
 */
void hal_i2c_WriteDataBlocking(hal_i2c_Port_t *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length)
{

    hal_i2c_SendStart(i2c_port);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_SB_Msk));

    hal_i2c_SendAddress(i2c_port, dev_address, false);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_ADDR_Msk));
    READ_REG(i2c_port->SR2);

    for(uint8_t index = 0; index < address_length; index++)
    {

        hal_i2c_SendByte(i2c_port, reg_address[index]);
        while(!READ_BIT(i2c_port->SR1, I2C_SR1_TXE_Msk));

    }
    for(uint8_t index = 0; index < data_length; index++)
    {

        hal_i2c_SendByte(i2c_port, data[index]);
        while(!READ_BIT(i2c_port->SR1, I2C_SR1_TXE_Msk));

    }

    while(!READ_BIT(i2c_port->SR1, I2C_SR1_BTF_Msk));
    hal_i2c_SendStop(i2c_port);

}

/**
 * @brief Read data from the I2C bus in a blocking manner
 * @param i2c_port       Peripheral base address for the I2C port of interest
 * @param dev_address    Device address to read from
 * @param reg_address    Register address to read from
 * @param address_length Length of the register address
 * @param data           Data to read from the specified register address
 * @param data_length    Length of the data to be read
 */
void hal_i2c_ReadDataBlocking(hal_i2c_Port_t *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length)
{

    hal_i2c_SendStart(i2c_port);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_SB_Msk));

    hal_i2c_SendAddress(i2c_port, dev_address, HAL_I2C_RWBIT_WRITE);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_ADDR_Msk));
    READ_REG(i2c_port->SR2);

    for(uint8_t index = 0; index < address_length; index++)
    {

        hal_i2c_SendByte(i2c_port, reg_address[index]);
        while(!READ_BIT(i2c_port->SR1, I2C_SR1_BTF_Msk));
        READ_REG(i2c_port->SR2);

    }

    hal_i2c_SendStart(i2c_port);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_SB_Msk));

    hal_i2c_SendAddress(i2c_port, dev_address, HAL_I2C_RWBIT_READ);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_ADDR_Msk));
    READ_REG(i2c_port->SR2);

    for(uint8_t index = 0; index < data_length; index++)
    {

        while(!READ_BIT(i2c_port->SR1, I2C_SR1_RXNE_Msk));
        data[index] = hal_i2c_ReceiveByte(i2c_port);

    }

    hal_i2c_SendStop(i2c_port);

}


/*********************
 * Private Functions *
 *********************/

/**
 * @brief Send a start bit
 * @param i2c_port Peripheral base address for the I2C port of interest
 */
void hal_i2c_SendStart(hal_i2c_Port_t *i2c_port)
{

    SET_BIT(i2c_port->CR1, I2C_CR1_START_Msk);

}

/**
 * @brief Send a stop bit
 * @param i2c_port Peripheral base address for the I2C port of interest
 */
void hal_i2c_SendStop(hal_i2c_Port_t *i2c_port)
{

    SET_BIT(i2c_port->CR1, I2C_CR1_STOP_Msk);

}

/**
 * @brief Send a byte to the I2C bus
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @param data     Byte to send
 */
void hal_i2c_SendByte(hal_i2c_Port_t *i2c_port, uint8_t data)
{

    WRITE_REG(i2c_port->DR, data);

}

/**
 * @brief Send a device address
 * @param i2c_port    Peripheral base address for the I2C port of interest
 * @param address     Address to be sent
 * @param read_enable Read/write bit
 */
void hal_i2c_SendAddress(hal_i2c_Port_t *i2c_port, uint8_t address, bool read_enable)
{

    hal_i2c_SendByte(i2c_port, (address << 1) | read_enable);

}

/**
 * @brief Read a byte from the I2C bus
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @return Byte read from the data register
 */
uint8_t hal_i2c_ReceiveByte(hal_i2c_Port_t *i2c_port)
{

    return (uint8_t)READ_REG(i2c_port->DR);

}

