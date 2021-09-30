

/**
 * @file    hal_usart.c
 * @brief   STM32F4 USART peripheral HAL implementation source file
 */


#include "hal_usart.h"


/****************************
/* Private Type Definitions *
 ****************************/

/* Transmit State Machine Enumerations */
enum hal_usart_TransmitState
{
    TRANSMITTING,
    IDLE
}

/* Receive State Machine Enumerations */
enum hal_usart_ReceiveState
{
    RECEIVING,
    IDLE
}

/* USART Peripheral State Struct */
typedef struct hal_usart_StateStruct
{
    enum hal_usart_TransmitState transmit_state;
    char                         *transmit_buffer;
    uint32_t                     transmit_length;
    uint32_t                     transmit_index;
    enum hal_usart_ReceiveState  receive_state;
    char                         *receive_buffer;
    uint32_t                     receive_index;
    uint32_t                     receive_length;
}hal_usart_StateStruct;


/*******************************
/* Private Function Prototypes *
 *******************************/

static uint8_t hal_usart_GetPeripheralIndex(hal_usart_Port *usart_port);
static IRQn_Type hal_usart_GetPeripheralIRQ(hal_usart_Port *usart_port);
static void hal_usart_GenericIRQHandler(hal_usart_Port *usart_port);


/*********************
/* Private Variables *
 *********************/

/* USART Peripheral State Table */
static hal_usart_StateStruct state_table[HAL_USART_NUM_PERIPHERALS];


/********************
/* Public Functions *
 ********************/

/**
 * @brief Configure the specified USART port
 * @param usart_port Peripheral base address for the USART port of interest
 * @param config     Pointer to a struct containing the USART port configuration parameters
 */
void hal_usart_Init(hal_usart_Port *usart_port, hal_usart_ConfigStruct *config)
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

    MODIFY_REG(usart_port->CR2, HAL_USART_CR2_MASK, (config->StopBits << HAL_USART_CR2_STOP_POSITION));

    uint8_t peripheral_index = hal_usart_GetPeripheralIndex(*usart_port);

    state_table[peripheral_index]->transmit_state = IDLE;
    state_table[peripheral_index]->receive_state = IDLE;

    if(config->ReceiveEnable)
    {

        state_table[peripheral_index]->receive_buffer = \
        (char *)calloc(HAL_USART_DEFAULT_RECEIVE_BUFFER_LENGTH, sizeof(char));

    }

    __NVIC_EnableIRQ(hal_usart_GetPeripheralIRQ(*usart_port))

}

/**
 * @brief Transmit data over the USART port specified in a blocking mode
 * @param usart_port Peripheral base address for the USART port of interest
 * @param data       Pointer to an array of characters to transmit
 * @param length     Number of bytes to transmit
 */
void hal_usart_TransmitBlocking(hal_usart_Port *usart_port, char *data, uint32_t length)
{

    for(uint32_t data_index = 0; data_index < length; data_index++)
    {

        while(!READ_BIT(usart_port->SR, HAL_USART_SR_TXE));
        WRITE_REG(usart_port->DR, data[data_index]);

    }

}

/**
 * @brief Transmit data over the USART port specified in a non-blocking mode
 * @param usart_port Peripheral base address for the USART port of interest
 * @param data       Pointer to an array of characters to transmit
 * @param length     Number of bytes to transmit
 */
void hal_usart_TransmitNonBlocking(hal_usart_Port *usart_port, char *data, uint32_t length)
{

    uint8_t peripheral_index = hal_usart_GetPeripheralIndex(*usart_port);

    state_table[peripheral_index]->transmit_buffer = data;
    state_table[peripheral_index]->transmit_length = length;

    WRITE_REG(usart_port->DR, data[0]);

    state_table[peripheral_index]->transmit_index = 1;
    state_table[peripheral_index]->transmit_state = TRANSMITTING;

}


/*********************
/* Private Functions *
 *********************/

/**
 * @brief Return the peripheral state table index corresponding to the specified USART port
 * @param usart_port Peripheral base address for the USART port of interest
 * @return Peripheral state table index for the specified USART port
 */
static uint8_t hal_usart_GetPeripheralIndex(hal_usart_Port *usart_port)
{

    switch(usart_port)
    {

        case USART1:
            return 0;

        case USART2:
            return 1;

        case USART3:
            return 2;

        case USART4:
            return 3;
            
        case USART5:
            return 4;

        case USART6:
            return 5;

    }

}

/**
 * @brief Return the interrupt vector entry corresponding to the specified USART port
 * @param usart_port Peripheral base address for the USART port of interest
 * @return Interrupt vector entry for the specified USART port
 */
static IRQn_Type hal_usart_GetPeripheralIRQ(hal_usart_Port *usart_port)
{

    switch(usart_port)
    {

        case USART1:
            return USART1_IRQn;

        case USART2:
            return USART2_IRQn;

        case USART3:
            return USART3_IRQn;

        case USART4:
            return USART4_IRQn;
            
        case USART5:
            return USART5_IRQn;

        case USART6:
            return USART6_IRQn;

    }

}


/******************************
/* Interrupt Service Routines *
 ******************************/

/**
 * @brief Generic interrupt handler for a USART port, containing a simple state machine for non-blocking transmit
 * @param usart_port Peripheral base address for the USART port of interest
 */
static void hal_usart_GenericIRQHandler(hal_usart_Port *usart_port)
{

    uint32_t usart_status = READ_REG(usart_port->SR);
    uint8_t peripheral_index = hal_usart_GetPeripheralIndex(*usart_port);


    if(state_table[peripheral_index]->transmit_state == TRANSMITTING)
    {

        if(usart_status && HAL_USART_SR_TXE_MASK)
        {

            if(state_table[peripheral_index]->transmit_index < state_table[peripheral_index]->transmit_length)
            {

                WRITE_REG(usart_port->DR, state_table[peripheral_index]->transmit_buffer[state_table[peripheral_index]->transmit_buffer]);
                state_table[peripheral_index]->transmit_index++;

            }
            else
            {

                state_table[peripheral_index]->transmit_buffer = NULL;
                state_table[peripheral_index]->transmit_state = IDLE;

            }

        }

    }

}

/**
 * @brief USART1 interrupt handler
 */
void USART1_IRQHandler(void)
{

    hal_usart_GenericIRQHandler(USART1);

}

/**
 * @brief USART2 interrupt handler
 */
void USART2_IRQHandler(void)
{

    hal_usart_GenericIRQHandler(USART2);

}

/**
 * @brief USART3 interrupt handler
 */
void USART3_IRQHandler(void)
{

    hal_usart_GenericIRQHandler(USART3);

}

/**
 * @brief USART4 interrupt handler
 */
void USART4_IRQHandler(void)
{

    hal_usart_GenericIRQHandler(USART4);

}

/**
 * @brief USART5 interrupt handler
 */
void USART5_IRQHandler(void)
{

    hal_usart_GenericIRQHandler(USART5);

}

/**
 * @brief USART6 interrupt handler
 */
void USART6_IRQHandler(void)
{

    hal_usart_GenericIRQHandler(USART6);

}

