

/**
 * @file    hal_usart.h
 * @brief   STM32F4 USART peripheral HAL implementation header file
 */


#ifndef __HAL_USART
#define __HAL_USART


/************
/* Includes *
 ************/

#include "stm32f4xx.h"


/***********
/* Defines *
 ***********/

/* USART_SR Register Mask/Position */
#define HAL_USART_SR_TXE_MASK     HAL_USART_SR_TXE_Msk  // Transmit data register empty mask
#define HAL_USART_SR_TXE_POSITION HAL_USART_SR_TXE_Pos  // Transmit data register empty position

/* USART_BRR Register Mask/Position */
#define HAL_USART_BRR_DIV_FRACTION_MASK USART_BRR_DIV_Fraction_Msk  // Baud rate divider fraction mask
#define HAL_USART_BRR_DIV_MANTISSA_MASK USART_BRR_DIV_Mantissa_Msk  // Baud rate divider mantissa mask

#define HAL_USART_BRR_DIV_FRACTION_POSITION USART_BRR_DIV_Fraction_Pos  // Baud rate divider fraction position
#define HAL_USART_BRR_DIV_MANTISSA_POSITION USART_BRR_DIV_Mantissa_Pos  // Baud rate divider mantissa position

/* USART_CR1 Register Mask/Position */
#define HAL_USART_CR1_RE_MASK     USART_CR1_RE_Msk      // Receiver enable mask
#define HAL_USART_CR1_TE_MASK     USART_CR1_TE_Msk      // Transmitter enable mask
#define HAL_USART_CR1_RXNEIE_MASK USART_CR1_RXNEIE_Msk  // RXNE interrupt enable mask
#define HAL_USART_CR1_TXEIE_MASK  USART_CR1_TXEIE_Msk   // TXE interrupt enable mask
#define HAL_USART_CR1_PS_MASK     USART_CR1_PS_Msk      // Parity selection mask
#define HAL_USART_CR1_PCE_MASK    USART_CR1_PCE_Msk     // Parity control enable mask
#define HAL_USART_CR1_M_MASK      USART_CR1_M_Msk       // Word length mask
#define HAL_USART_CR1_UE_MASK     USART_CR1_UE_Msk      // USART enable mask
#define HAL_USART_CR1_OVER8_MASK  USART_CR1_OVER8_Msk   // Oversampling mode mask

#define HAL_USART_CR1_RE_POSITION     USART_CR1_RE_Pos      // Receiver enable position
#define HAL_USART_CR1_TE_POSITION     USART_CR1_TE_Pos      // Transmitter enable position
#define HAL_USART_CR1_RXNEIE_POSITION USART_CR1_RXNEIE_Pos  // RXNE interrupt enable position
#define HAL_USART_CR1_TXEIE_POSITION  USART_CR1_TXEIE_Pos   // TXE interrupt enable position
#define HAL_USART_CR1_PS_POSITION     USART_CR1_PS_Pos      // Parity selection position
#define HAL_USART_CR1_PCE_POSITION    USART_CR1_PCE_Pos     // Parity control enable position
#define HAL_USART_CR1_M_POSITION      USART_CR1_M_Pos       // Word length position
#define HAL_USART_CR1_UE_POSITION     USART_CR1_UE_Pos      // USART enable position
#define HAL_USART_CR1_OVER8_POSITION  USART_CR1_OVER8_Pos   // Oversampling mode position

/* USART_CR2 Register Mask/Position */
#define HAL_USART_CR2_STOP_MASK     HAL_USART_CR2_STOP_Msk  // Stop bits mask
#define HAL_USART_CR2_STOP_POSITION HAL_USART_CR2_STOP_Pos  // Stop bits position

/* USART_CR1_PS (Parity Selection) Enumerations */
#define HAL_USART_CR1_PS_EVEN (0x0)  // Even parity
#define HAL_USART_CR1_PS_ODD  (0x1)  // Odd parity

/* USART_CR1_M (Word Length) Enumerations */
#define HAL_USART_CR1_M_8 (0x0)  // 8-bit word
#define HAL_USART_CR1_M_9 (0x1)  // 9-bit word

/* USART_CR1_OVER8 (Oversampling Mode) Enumerations */
#define HAL_USART_CR1_OVER8_16 (0x0)  // Oversampling by 8
#define HAL_USART_OVER8_8      (0x1)  // Oversampling by 16

/* USART_CR2_STOP (Number of Stop Bits) Enumerations */
#define HAL_USART_CR2_STOP_1   (0x0)  // 1 stop bit
#define HAL_USART_CR2_STOP_0_5 (0x1)  // 0.5 stop bits
#define HAL_USART_CR2_STOP_2   (0x2)  // 2 stop bits
#define HAL_USART_CR2_STOP_1_5 (0x3)  // 1.5 stop bits

/* USART_BRR Config Mask */
#define HAL_USART_BRR_MASK (USART_BRR_DIV_MANTISSA_MASK | USART_BRR_DIV_FRACTION_MASK)

/* USART_CR1 Config Mask */
#define HAL_USART_CR1_MASK (USART_CR1_TE_MASK | \
                            USART_CR1_RE_MASK | \
                            USART_CR1_PCE_MASK | \
                            USART_CR1_PS_MASK | \
                            USART_CR1_M_MASK | \
                            USART_CR1_UE_MASK | \
                            USART_CR1_OVER8_MASK)

/* Defaults */
#define HAL_USART_DEFAULT_RECEIVE_BUFFER_LENGTH 256  // Default length of the receive buffer
#define HAL_USART_NUM_PERIPHERALS               6    // Total number of USART instances available


/***************************
/* Public Type Definitions *
 ***************************/

/* USART Port Definition */
typedef USART_TypeDef HAL_USART_Port;

/* USART Configuration Field Definitions */
typedef uint16_t HAL_USART_BaudMantissa;
typedef uint8_t  HAL_USART_BaudFraction;
typedef uint8_t  HAL_USART_ParitySelection;
typedef uint8_t  HAL_USART_WordLength;
typedef uint8_t  HAL_USART_OversamplingMode;
typedef uint8_t  HAL_USART_StopBits;
typedef bool     HAL_USART_TransmitEnable;
typedef bool     HAL_USART_ReceiveEnable;
typedef bool     HAL_USART_ParityControlEnable;
typedef bool     HAL_USART_USARTEnable;

/* USART Peripheral Instance Definition */
typedef struct HAL_USART_ConfigStruct
{
    HAL_USART_BaudMantissa        BaudMantissa;         // Baud rate divider mantissa
    HAL_USART_BaudFraction        BaudFraction;         // Baud rate divider fraction
    HAL_USART_TransmitEnable      TransmitEnable;       // USART transmitter enable
    HAL_USART_ReceiveEnable       ReceiveEnable;        // USART receiver enable
    HAL_USART_ParityControlEnable ParityControlEnable;  // Parity control enable
    HAL_USART_ParitySelection     ParitySelection;      // Parity mode
    HAL_USART_WordLength          WordLength;           // Word length
    HAL_USART_USARTEnable         USART_Enable;         // USART peripheral enable
    HAL_USART_OversamplingMode    OversamplingMode;     // Oversampling mode
    HAL_USART_StopBits            StopBits;             // Number of stop bits
}HAL_USART_ConfigStruct;


/******************************
/* Public Function Prototypes *
 ******************************/

void HAL_USART_Init(HAL_USART_Port *usart_port, HAL_USART_ConfigStruct *config);
void HAL_USART_TransmitBlocking(HAL_USART_Port *usart_port, char *data, uint32_t length);
void HAL_USART_TransmitNonBlocking(HAL_USART_Port *usart_port, char *data, uint32_t length);


#endif /* __HAL_USART */

