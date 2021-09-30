

/**
 * @file    hal_i2c.c
 * @brief   STM32F4 I2C peripheral HAL implementation source file
 */


/************
/* Includes *
 ************/

#include "hal_i2c.h"


/****************************
/* Private Type Definitions *
 ****************************/

/* Write State Machine Enumerations */
enum HAL_I2C_PeripheralState
{
    READING,
    WRITING,
    IDLE
}

/* Write State Machine Enumerations */
enum HAL_I2C_WriteState
{
    START_SENT,
    ADDRESS_SENT,
    REGISTER_SENDING,
    DATA_SENDING,
    DATA_SENT,
    STOP_SENT,
    IDLE
}

/* Read State Machine Enumerations */
enum HAL_I2C_ReadState
{
    START_1_SENT,
    ADDRESS_1_SENT,
    REGISTER_SENDING,
    REGISTER_SENT,
    START_2_SENT,
    ADDRESS_2_SENT,
    DATA_RECEIVING,
    STOP_SENT,
    IDLE
}

/* I2C Peripheral State Struct */
typedef struct HAL_I2C_StateStruct
{
    enum HAL_I2C_PeripheralState peripheral_state;         // Peripheral state (Read/write)
    enum HAL_I2C_WriteState      write_state;              // Peripheral write state
    enum HAL_I2C_ReadState       read_state;               // Peripheral read state
    uint8_t                      device_address;           // Target device address
    uint8_t                      *register_address;        // I2C read/write address pointer
    uint8_t                      register_address_length;  // I2C read/write register address length
    uint8_t                      *data;                    // Read/write data pointer
    uint8_t                      data_length;              // Read/write data length
    uint8_t                      index;                    // Current read/write address/data index
}


/*******************************
/* Private Function Prototypes *
 *******************************/

static void HAL_I2C_SendStart(HAL_I2C_Port *i2c_port);
static void HAL_I2C_SendAddress(HAL_I2C_Port *i2c_port, uint8_t address, bool read_enable);
static void HAL_I2C_SendByte(HAL_I2C_Port *i2c_port, uint8_t data);
static uint8_t HAL_I2C_ReceiveByte(HAL_I2C_Port *i2c_port);
static void HAL_I2C_SendStop(HAL_I2C_Port *i2c_port);
static uint8_t HAL_I2C_GetPeripheralIndex(HAL_I2C_Port *i2c_port);
static IRQn_Type HAL_I2C_GetPeripheralERIRQ(HAL_I2C_Port *i2c_port);
static IRQn_Type HAL_I2C_GetPeripheralEVIRQ(HAL_I2C_Port *i2c_port);
static void HAL_I2C_GenericEVIRQHandler(HAL_I2C_Port *i2c_port);
static void HAL_I2C_StateMachine(HAL_I2C_Port *i2c_port);


/*********************
/* Private Variables *
 *********************/

/* I2C Peripheral State Table */
static HAL_I2C_StateStruct state_table[HAL_I2C_NUM_PERIPHERALS];


/********************
/* Public Functions *
 ********************/

/**
 * @brief Configure the specified I2C port
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @param config   Pointer to a struct containing the I2C port configuration parameters
 */
void HAL_I2C_Init(HAL_I2C_Port *i2c_port, HAL_I2C_ConfigStruct *config)
{

    SET_BIT(i2c_port->CR1, HAL_I2C_CR1_SWRST_MASK);
    CLEAR_BIT(i2c_port->CR1, HAL_I2C_CR1_SWRST_MASK);
    CLEAR_BIT(i2c_port->CR1, HAL_I2C_CR1_PE_MASK);

    if(config->FastMode)
    {

        MODIFY_REG(i2c_port->CCR, HAL_I2C_CCR_MASK, (uint32_t)(HAL_I2C_CCR_FS_Msk | (config->APB_ClockFrequency / 800000)));
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

    uint8_t peripheral_index = HAL_I2C_GetPeripheralIndex(*i2c_port);
    state_table[peripheral_index]->peripheral_state = IDLE;
    state_table[peripheral_index]->write_state = IDLE;
    state_table[peripheral_index]->read_state = IDLE;

}

/**
 * @brief Check if the I2C bus is busy
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @return True if the peripheral is busy
 */
bool HAL_I2C_IsBusy(HAL_I2C_Port *i2c_port)
{

    return (bool)READ_BIT(i2c_port->SR2, I2C_SR2_BUSY_Msk);

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
void HAL_I2C_WriteDataBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length)
{

    uint8_t peripheral_index = HAL_I2C_GetPeripheralIndex(*i2c_port);

    state_table[peripheral_index]->peripheral_state = WRITING;

    HAL_I2C_SendStart(i2c_port);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_SB_Msk));

    HAL_I2C_SendAddress(i2c_port, dev_address, false);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_ADDR_Msk));
    READ_REG(i2c_port->SR2);

    for(uint8_t index = 0; index < address_length; index++)
    {

        HAL_I2C_SendByte(i2c_port, reg_address[index]);
        while(!READ_BIT(i2c_port->SR1, I2C_SR1_TXE_Msk));

    }
    for(uint8_t index = 0; index < data_length; index++)
    {

        HAL_I2C_SendByte(i2c_port, data[index]);
        while(!READ_BIT(i2c_port->SR1, I2C_SR1_TXE_Msk));

    }

    while(!READ_BIT(i2c_port->SR1, I2C_SR1_BTF_Msk));
    HAL_I2C_SendStop(i2c_port);

    state_table[peripheral_index]->peripheral_state = IDLE;

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
void HAL_I2C_ReadDataBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length)
{

    uint8_t peripheral_index = HAL_I2C_GetPeripheralIndex(*i2c_port);

    state_table[peripheral_index]->peripheral_state = READING;

    HAL_I2C_SendStart(i2c_port);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_SB_Msk));

    HAL_I2C_SendAddress(i2c_port, dev_address, HAL_I2C_RWBIT_WRITE);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_ADDR_Msk));
    READ_REG(i2c_port->SR2);

    for(uint8_t index = 0; index < address_length; index++)
    {

        HAL_I2C_SendByte(i2c_port, reg_address[index]);
        while(!READ_BIT(i2c_port->SR1, I2C_SR1_BTF_Msk));
        READ_REG(i2c_port->SR2);

    }

    HAL_I2C_SendStart(i2c_port);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_SB_Msk));

    HAL_I2C_SendAddress(i2c_port, dev_address, HAL_I2C_RWBIT_READ);
    while(!READ_BIT(i2c_port->SR1, I2C_SR1_ADDR_Msk));
    READ_REG(i2c_port->SR2);

    for(uint8_t index = 0; index < data_length; index++)
    {

        while(!READ_BIT(i2c_port->SR1, I2C_SR1_RXNE_Msk));
        data[index] = HAL_I2C_RecieveByte(i2c_port);

    }

    HAL_I2C_SendStop(i2c_port);

    state_table[peripheral_index]->peripheral_state = IDLE;

}

/**
 * @brief Write data to the I2C bus in a non-blocking manner
 * @param i2c_port       Peripheral base address for the I2C port of interest
 * @param dev_address    Device address to write to
 * @param reg_address    Register address to write to
 * @param address_length Length of the register address
 * @param data           Data to write to the specified register address
 * @param data_length    Length of the data to be written
 */
void HAL_I2C_WriteDataNonBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length)
{

    uint8_t peripheral_index = HAL_I2C_GetPeripheralIndex(*i2c_port);

    state_table[peripheral_index]->peripheral_state = WRITING;
    state_table[peripheral_index]->device_address = dev_address;
    state_table[peripheral_index]->register_address = reg_address;
    state_table[peripheral_index]->register_address_length = address_length;
    state_table[peripheral_index]->data = data;
    state_table[peripheral_index]->data_length = data_length;
    state_table[peripheral_index]->index = 0;

    __NVIC_EnableIRQ(HAL_I2C_GetPeripheralEVIRQ(i2c_port));

    HAL_I2C_SendStart(i2c_port);

    state_table[peripheral_index]->write_state = START_SENT;

}

/**
 * @brief Read data from the I2C bus in a non-blocking manner
 * @param i2c_port       Peripheral base address for the I2C port of interest
 * @param dev_address    Device address to read from
 * @param reg_address    Register address to read from
 * @param address_length Length of the register address
 * @param data           Data to read from the specified register address
 * @param data_length    Length of the data to be read
 */
void HAL_I2C_ReadDataNonBlocking(HAL_I2C_Port *i2c_port, uint8_t dev_address, uint8_t *reg_address, uint8_t address_length, uint8_t *data, uint8_t data_length)
{

    uint8_t peripheral_index = HAL_I2C_GetPeripheralIndex(*i2c_port);

    state_table[peripheral_index]->peripheral_state = READING;
    state_table[peripheral_index]->device_address = dev_address;
    state_table[peripheral_index]->register_address = reg_address;
    state_table[peripheral_index]->register_address_length = address_length;
    state_table[peripheral_index]->data = data;
    state_table[peripheral_index]->data_length = data_length;
    state_table[peripheral_index]->index = 0;

    __NVIC_EnableIRQ(HAL_I2C_GetPeripheralEVIRQ(i2c_port));

    HAL_I2C_SendStart(i2c_port);

    state_table[peripheral_index]->write_state = START_1_SENT;

}


/*********************
/* Private Functions *
 *********************/

/**
 * @brief Send a start bit
 * @param i2c_port Peripheral base address for the I2C port of interest
 */
void HAL_I2C_SendStart(HAL_I2C_Port *i2c_port)
{

    SET_BIT(i2c_port->CR1, I2C_CR1_START_Msk);

}

/**
 * @brief Send a stop bit
 * @param i2c_port Peripheral base address for the I2C port of interest
 */
void HAL_I2C_SendStop(HAL_I2C_Port *i2c_port)
{

    SET_BIT(i2c_port->CR1, I2C_CR1_STOP_Msk);

}

/**
 * @brief Send a byte
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @param data     Byte to send
 */
void HAL_I2C_SendByte(HAL_I2C_Port *i2c_port, uint8_t data)
{

    WRITE_REG(i2c_port->DR, data);

}

/**
 * @brief Send a device address
 * @param i2c_port    Peripheral base address for the I2C port of interest
 * @param address     Address to be sent
 * @param read_enable Read/write bit
 */
void HAL_I2C_SendAddress(HAL_I2C_Port *i2c_port, uint8_t address, bool read_enable)
{

    HAL_I2C_SendByte(i2c_port, (address << 1) | read_enable);

}

/**
 * @brief Read a byte
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @return Byte read from the data register
 */
uint8_t HAL_I2C_RecieveByte(HAL_I2C_Port *i2c_port)
{

    return (uint8_t)READ_REG(i2c_port->DR);

}

/**
 * @brief Return the peripheral state table index corresponding to the specified I2C port
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @return Peripheral state table index for the specified I2C port
 */
static uint8_t HAL_I2C_GetPeripheralIndex(HAL_I2C_Port *i2c_port)
{

    switch(i2c_port)
    {

        case I2C1:
            return 0;

        case I2C2:
            return 1;

        case I2C3:
            return 2;

    }

}

/**
 * @brief Return the event interrupt vector entry corresponding to the specified I2C port
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @return Interrupt vector entry for the specified I2C port
 */
static IRQn_Type HAL_I2C_GetPeripheralEVIRQ(HAL_I2C_Port *i2c_port)
{

    switch(i2c_port)
    {

        case I2C1:
            return I2C1_EV_IRQn;

        case I2C2:
            return I2C2_EV_IRQn;

        case I2C3:
            return I2C3_EV_IRQn;

    }

}

/**
 * @brief Return the error interrupt vector entry corresponding to the specified I2C port
 * @param i2c_port Peripheral base address for the I2C port of interest
 * @return Interrupt vector entry for the specified I2C port
 */
static IRQn_Type HAL_I2C_GetPeripheralERIRQ(HAL_I2C_Port *i2c_port)
{

    switch(i2c_port)
    {

        case I2C1:
            return I2C1_ER_IRQn;

        case I2C2:
            return I2C2_ER_IRQn;

        case I2C3:
            return I2C3_ER_IRQn;

    }

}

/**
 * @brief State machine to be run on I2C event interrupt if the peripheral is writing/reading
 * @param i2c_port Peripheral base address for the I2C port of interest
 */
static void HAL_I2C_StateMachine(HAL_I2C_Port *i2c_port)
{

    uint8_t peripheral_index = HAL_I2C_GetPeripheralIndex(*i2c_port);

    switch(state_table[peripheral_index]->peripheral_state)
    {

        case WRITING:
    
            switch(state_table[peripheral_index]->write_state)
            {

                case START_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_SB_MASK))
                    {

                        HAL_I2C_SendAddress(i2c_port, state_table[peripheral_index]->device_address, false);
                        state_table[peripheral_index]->write_state = ADDRESS_SENT;

                    }
                    break;

                case ADDRESS_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_ADDR_MASK))
                    {

                        READ_REG(i2c_port->SR2);
                        HAL_I2C_SendByte(i2c_port, state_table[peripheral_index]->register_address[0]);
                        state_table[peripheral_index]->index = 1;
                        state_table[peripheral_index]->write_state = REGISTER_SENDING;

                    }
                    break;

                case REGISTER_SENDING:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_TXE_MASK))
                    {

                        if(state_table[peripheral_index]->index < state_table[peripheral_index]->register_address_length)
                        {

                            HAL_I2C_SendByte(i2c_port, state_table[peripheral_index]->register_address[state_table[peripheral_index]->index]);
                            state_table[peripheral_index]->index++;

                        }
                        else
                        {

                            HAL_I2C_SendByte(i2c_port, state_table[peripheral_index]->data[0]);
                            state_table[peripheral_index]->index = 1;
                            state_table[peripheral_index]->register_address = NULL;
                            state_table[peripheral_index]->register_address_length = 0;
                            state_table[peripheral_index]->write_state = DATA_SENDING;

                        }

                    }
                    break;

                case DATA_SENDING:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_TXE_MASK))
                    {

                        if(state_table[peripheral_index]->index < state_table[peripheral_index]->data_length)
                        {

                            HAL_I2C_SendByte(i2c_port, state_table[peripheral_index]->data[state_table[peripheral_index]->index]);
                            state_table[peripheral_index]->index++;

                        }
                        else
                        {

                            state_table[peripheral_index]->index = 0;
                            state_table[peripheral_index]->data = NULL;
                            state_table[peripheral_index]->data_length = 0;
                            state_table[peripheral_index]->write_state = DATA_SENT;

                        }

                    }
                    break;

                case DATA_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_BTF_MASK))
                    {

                        HAL_I2C_SendStop(i2c_port);
                        state_table[peripheral_index]->write_state = STOP_SENT;

                    }
                    break;

                case STOP_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR2_BUSY_MASK))
                    {

                        state_table[peripheral_index]->write_state = IDLE;
                        state_table[peripheral_index]->peripheral_state=IDLE;

                    }
                    break;

                default:
                    break;

            }
            break;

        case READING:

            switch(state_table[peripheral_index]->read_state)
            {

                case START_1_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_SB_MASK))
                    {

                        HAL_I2C_SendAddress(i2c_port, state_table[peripheral_index]->device_address, false);
                        state_table[peripheral_index]->read_state = ADDRESS_SENT;

                    }
                    break;

                case ADDRESS_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_ADDR_MASK))
                    {

                        READ_REG(i2c_port->SR2);
                        HAL_I2C_SendByte(i2c_port, state_table[peripheral_index]->register_address[0]);
                        state_table[peripheral_index]->index = 1;
                        state_table[peripheral_index]->read_state = REGISTER_SENDING;

                    }
                    break;

                case REGISTER_SENDING:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_TXE_MASK))
                    {

                        if(state_table[peripheral_index]->index < state_table[peripheral_index]->register_address_length)
                        {

                            HAL_I2C_SendByte(i2c_port, state_table[peripheral_index]->register_address[state_table[peripheral_index]->index]);
                            state_table[peripheral_index]->index++;

                        }
                        else
                        {

                            HAL_I2C_SendByte(i2c_port, state_table[peripheral_index]->data[0]);
                            state_table[peripheral_index]->read_state = REGISTER_SENT;

                        }

                    }
                    break;

                case REGISTER_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_TXE_MASK))
                    {

                        HAL_I2C_SendStart(i2c_port);
                        state_table[peripheral_index]->read_state = START_2_SENT;


                    }
                    break;

                case START_2_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_SB_MASK))
                    {

                        HAL_I2C_SendAddress(i2c_port, state_table[peripheral_index]->device_address, false);
                        state_table[peripheral_index]->read_state = ADDRESS_2_SENT;

                    }
                    break;

                case ADDRESS_2_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_ADDR_MASK))
                    {

                        READ_REG(i2c_port->SR2);
                        state_table[peripheral_index]->register_address[0] = HAL_I2C_ReceiveByte(i2c_port);
                        state_table[peripheral_index]->index = 1;
                        state_table[peripheral_index]->read_state = DATA_RECEIVING;

                    }
                    break;

                case DATA_RECEIVING:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR1_RXNE_MASK))
                    {

                        if(state_table[peripheral_index]->index < state_table[peripheral_index]->data_length)
                        {

                            state_table[peripheral_index]->register_address[state_table[peripheral_index]->index] = HAL_I2C_ReceiveByte(i2c_port);
                            state_table[peripheral_index]->index++;

                        }
                        else
                        {

                            state_table[peripheral_index]->index = 0;
                            state_table[peripheral_index]->data = NULL;
                            state_table[peripheral_index]->data_length = 0;

                            HAL_I2C_SendStop(i2c_port);

                            state_table[peripheral_index]->read_state = STOP_SENT;

                        }

                    }
                    break;

                case STOP_SENT:

                    if(READ_BIT(i2c_port->SR1, HAL_I2C_SR2_BUSY_MASK))
                    {

                        state_table[peripheral_index]->read_state = IDLE;
                        state_table[peripheral_index]->peripheral_state=IDLE;

                    }
                    break;

                default:
                    break;

            }
            break;

        default:
            break;

    }

}


/******************************
/* Interrupt Service Routines *
 ******************************/

/**
 * @brief Generic interrupt handler for an I2C port, which jumps to a state machine
 * @param usart_port Peripheral base address for the USART port of interest
 */
static void HAL_I2C_GenericEVIRQHandler(HAL_I2C_Port *i2c_port)
{

    HAL_I2C_StateMachine(i2c_port);

}

/**
 * @brief I2C1 event interrupt handler
 */
void I2C1_EV_IRQHandler(void)
{

    HAL_I2C_GenericEVIRQHandler(I2C1);

}

/**
 * @brief I2C2 event interrupt handler
 */
void I2C2_EV_IRQHandler(void)
{

    HAL_I2C_GenericEVIRQHandler(I2C2);

}

/**
 * @brief I2C3 event interrupt handler
 */
void I2C3_EV_IRQHandler(void)
{

    HAL_I2C_GenericEVIRQHandler(I2C3);

}

