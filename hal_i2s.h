

/**
 * @file    hal_i2s.h
 * @brief   STM32F4 I2S peripheral HAL implementation header file
 */


#ifndef __HAL_I2S
#define __HAL_I2S


/************
 * Includes *
 ************/

#include "stm32f4xx.h"

#include "hal_dma.h"


/***********
 * Defines *
 ***********/

/* SPI_I2SCFGR Register Mask/Position */
#define HAL_I2S_I2SCFGR_I2SCFG_MASK  SPI_I2SCFGR_I2SCFG_Msk   // I2S configuration mode mask
#define HAL_I2S_I2SCFGR_I2SSTD_MASK  SPI_I2SCFGR_I2SSTD_Msk   // I2S standard mask
#define HAL_I2S_I2SCFGR_PCMSYNC_MASK SPI_I2SCFGR_PCMSYNC_Msk  // PCM sync mask
#define HAL_I2S_I2SCFGR_CKPOL_MASK   SPI_I2SCFGR_CKPOL_Msk    // Clock polarity mask
#define HAL_I2S_I2SCFGR_DATLEN_MASK  SPI_I2SCFGR_DATLEN_Msk  // Data length mask
#define HAL_I2S_I2SCFGR_CHLEN_MASK   SPI_I2SCFGR_CHLEN_Msk    // Channel length mask
#define HAL_I2S_I2SCFGR_I2SMOD_MASK  SPI_I2SCFGR_I2SMOD_Msk   // I2S/SPI peripheral mode mask
#define HAL_I2S_I2SCFGR_I2SE_MASK    SPI_I2SCFGR_I2SE_Msk     // I2S enable mask

#define HAL_I2S_I2SCFGR_I2SCFG_POSITION  SPI_I2SCFGR_I2SCFG_Pos   // I2S configuration mode position
#define HAL_I2S_I2SCFGR_I2SSTD_POSITION  SPI_I2SCFGR_I2SSTD_Pos   // I2S standard position
#define HAL_I2S_I2SCFGR_PCMSYNC_POSITION SPI_I2SCFGR_PCMSYNC_Pos  // PCM sync position
#define HAL_I2S_I2SCFGR_CKPOL_POSITION   SPI_I2SCFGR_CKPOL_Pos    // Clock polarity position
#define HAL_I2S_I2SCFGR_DATLEN_POSITON   SPI_I2SCFGR_DATLEN_Pos  // Data length positon
#define HAL_I2S_I2SCFGR_CHLEN_POSITON    SPI_I2SCFGR_CHLEN_Pos    // Channel length position
#define HAL_I2S_I2SCFGR_I2SMOD_POSITION  SPI_I2SCFGR_I2SMOD_Pos   // I2S/SPI peripheral mode position
#define HAL_I2S_I2SCFGR_I2SE_POSITION    SPI_I2SCFGR_I2SE_Pos     // I2S enable position

/* SPI_I2SPR Register Mask/Position */
#define HAL_I2S_I2SPR_I2SDIV_MASK SPI_I2SPR_I2SDIV_Msk  // I2S clock prescaler mask
#define HAL_I2S_I2SPR_ODD_MASK SPI_I2SPR_ODD_Msk        // I2S clock prescaler odd bit mask
#define HAL_I2S_I2SPR_MCKOE_MASK SPI_I2SPR_MCKOE_Msk    // I2S master clock enable mask

#define HAL_I2S_I2SPR_I2SDIV_POSITION SPI_I2SPR_I2SDIV_Pos  // I2S clock prescaler position
#define HAL_I2S_I2SPR_ODD_POSITION SPI_I2SPR_ODD_Pos        // I2S clock prescaler odd bit position
#define HAL_I2S_I2SPR_MCKOE_POSITION SPI_I2SPR_MCKOE_Pos    // I2S master clock enable position

/* SPI_SR Register Mask/Position */
#define HAL_I2S_SR_TXE_MASK     SPI_SR_TXE_Msk  // I2S transmit data register empty mask
#define HAL_I2S_SR_TXE_POSITION SPI_SR_TXE_Pos  // I2S transmit data register empty position

/* SPI_I2SCFGR_I2SCFG (I2S bus mode) Enumerations */
#define HAL_I2S_I2SCFGR_I2SCFG_SLAVETRANSMIT  (0x0)  // Slave transmit mode
#define HAL_I2S_I2SCFGR_I2SCFG_SLAVERECEIVE   (0x1)  // Slave receive mode
#define HAL_I2S_I2SCFGR_I2SCFG_MASTERTRANSMIT (0x2)  // Master transmit mode
#define HAL_I2S_I2SCFGR_I2SCFG_MASTERRECEIVE  (0x3)  // Master receive mode

/* SPI_I2SCFGR_PCMSYNC (PCM Frame Synchronization) Enumerations */
#define HAL_I2S_I2SCFGR_PCMSYNC_SHORT (0x0)  // Short frame synchronization
#define HAL_I2S_I2SCFGR_PCMSYNC_LONG  (0x1)  // Long frame synchronization

/* SPI_I2SCFGR_I2SSTD (I2S Standard Selection) Enumerations */
#define HAL_I2S_I2SCFGR_I2SSTD_PHILLIPS       (0x0)  // I2S Phillips standard
#define HAL_I2S_I2SCFGR_I2SSTD_LEFTJUSTIFIED  (0x1)  // I2S left justified
#define HAL_I2S_I2SCFGR_I2SSTD_RIGHTJUSTIFIED (0x2)  // I2S right justified
#define HAL_I2S_I2SCFGR_I2SSTD_PCM            (0x3)  // I2S PCM standard

/* SPI_I2SCFGR_CKPOL (Clock Polarity) Enumerations */
#define HAL_I2S_I2SCFGR_CKPOL_LOW  (0x0)  // Default clock state low
#define HAL_I2S_I2SCFGR_CKPOL_HIGH (0x1)  // Default clock state high

/* SPI_I2SCFGR_DATLEN (Data Length) Enumerations */
#define HAL_I2S_I2SCFGR_DATLEN_16 (0x0)  // 16-bit audio data width
#define HAL_I2S_I2SCFGR_DATLEN_24 (0x1)  // 24-bit audio data width
#define HAL_I2S_I2SCFGR_DATLEN_32 (0x2)  // 32-bit audio data width

/* SPI_I2SCFGR_CHLEN (Data Length per Channel) Enumerations */
#define HAL_I2S_I2SCFGR_CHLEN_16 (0x0)  // 16-bit audio data width per channel
#define HAL_I2S_I2SCFGR_CHLEN_32 (0x1)  // 32-bit audio data width per channel

/* SPI_I2SCFGR Configuration Mask */
#define HAL_I2S_I2SCFGR_MASK (HAL_I2S_I2SCFGR_I2SCFG_MASK | \
                              HAL_I2S_I2SCFGR_I2SSTD_MASK | \
                              HAL_I2S_I2SCFGR_PCMSYNC_MASK | \
                              HAL_I2S_I2SCFGR_CKPOL_MASK | \
                              HAL_I2S_I2SCFGR_DATLEN_MASK | \
                              HAL_I2S_I2SCFGR_CHLEN_MASK | \
                              HAL_I2S_I2SCFGR_I2SMOD_MASK)

/* SPI_I2SPR Configuration Mask */
#define HAL_I2S_I2SPR_MASK (HAL_I2S_I2SPR_MCKOE_MASK | \
                            HAL_I2S_I2SPR_ODD_MASK | \
                            HAL_I2S_I2SPR_I2SDIV_POSITION)


/***************************
 * Public Type Definitions *
 ***************************/

/* I2S Port Definition */
typedef SPI_TypeDef hal_i2s_Port_t;

/* I2S Configuration Parameters */
typedef uint8_t hal_i2s_BusMode_t;
typedef uint8_t hal_i2s_PCMSync_t;
typedef uint8_t hal_i2s_Standard_t;
typedef uint8_t hal_i2s_ClockPolarity_t;
typedef uint8_t hal_i2s_DataLength_t;
typedef uint8_t hal_i2s_ChannelLength_t;

/* I2S Configuration Struct */
typedef struct hal_i2s_ConfigStruct_s
{
    hal_i2s_BusMode_t       BusMode;             // Bus mode (e.g. master transmit)
    hal_i2s_PCMSync_t       PCMSync;             // PCM frame sync setting
    hal_i2s_Standard_t      Standard;            // I2S standard to follow (e.g. Phillips, RJ,LJ)
    hal_i2s_ClockPolarity_t ClockPolarity;       // I2S bus clock state during idle (High/low)
    hal_i2s_DataLength_t    DataLength;          // Bite pser data transfer
    hal_i2s_ChannelLength_t ChannelLength;       // Bits per channel
    bool                    MasterClockEnable;   // I2S master clock enable
    bool                    OddPrescalerEnable;  // Add 1 to the prescaler value
    uint8_t                 LinearPrescaler;     // I2C clock prescaler = (LinearPrescaler * 2)
    bool                    I2SEnable;           // I2S bus enable
}hal_i2s_ConfigStruct_s;


/******************************
 * Public Function Prototypes *
 ******************************/

void hal_i2s_Init(hal_i2s_Port_t *i2s_port, hal_i2s_ConfigStruct_s *config);
void hal_i2s_WriteData16Blocking(hal_i2s_Port_t *i2s_port, int16_t *data, uint32_t length);
void hal_i2s_WriteData16NonBlocking(hal_i2s_Port_t *i2s_port, hal_dma_Stream_t *dma_stream, hal_dma_Channel_t dma_channel, int16_t *data, uint32_t length);
void hal_i2s_StopWrite(hal_dma_Stream_t *dma_stream);


#endif /* __HAL_I2S */

