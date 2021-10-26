

/**
 * @file    hal_dma.c
 * @brief   STM32F4 DMA peripheral HAL implementation source file
 */


/************
 * Includes *
 ************/

#include "hal_dma.h"


/********************
 * Public Functions *
 ********************/

/**
 * @brief Configure the specified DMA stream
 * @param dma_stream The DMA stream to be configured
 * @param config     Pointer to the configuration parameter struct
 */
void hal_dma_Init(hal_dma_Stream_t *dma_stream, hal_dma_ConfigStruct_s *config)
{

    hal_dma_Disable(dma_stream);

    MODIFY_REG(dma_stream->CR, HAL_DMA_SxCR_CONFIGMASK, ((config->FlowController << HAL_DMA_SxCR_PFCTRL_POSITION) | \
                                                         (config->TransferDirection << HAL_DMA_SxCR_DIR_POSITION) | \
                                                         (config->CircularMode << HAL_DMA_SxCR_CIRC_POSITION) | \
                                                         (config->PeripheralIncrementMode << HAL_DMA_SxCR_PINC_POSITION) | \
                                                         (config->MemoryIncrementMode << HAL_DMA_SxCR_MINC_POSITION) | \
                                                         (config->PeripheralDataSize << HAL_DMA_SxCR_PSIZE_POSITION) | \
                                                         (config->MemoryDataSize << HAL_DMA_SxCR_MSIZE_POSITION) | \
                                                         (config->PeripheralIncrementOffsetSize << HAL_DMA_SxCR_PINCOS_POSITION) | \
                                                         (config->PriorityLevel << HAL_DMA_SxCR_PL_POSITION) | \
                                                         (config->DoubleBufferMode << HAL_DMA_SxCR_DBM_POSITION) | \
                                                         (config->CurrentTarget << HAL_DMA_SxCR_CT_POSITION) | \
                                                         (config->PeripheralBurstMode << HAL_DMA_SxCR_PBURST_POSITION) | \
                                                         (config->MemoryBurstMode << HAL_DMA_SxCR_MBURST_POSITION) | \
                                                         (config->ChannelSelection << HAL_DMA_SxCR_CHSEL_POSITION)));

}

/**
 * @brief Setup the transfer parameters for the DMA stream
 * @param dma_stream              The DMA stream to be configured
 * @param peripheral_base_address Base address of the peripheral receiving the stream
 * @param memory_address          Memory address to pull data from
 * @param length                  Number of beats before stream termination/reset
 */
void hal_dma_ConfigureTransfer(hal_dma_Stream_t *dma_stream, uint32_t peripheral_base_address, uint32_t memory_address, uint16_t length)
{

    WRITE_REG(dma_stream->PAR, peripheral_base_address);
    WRITE_REG(dma_stream->M0AR, memory_address);
    WRITE_REG(dma_stream->NDTR, length);

}

/**
 * @brief Enmable the DMA stream
 * @param dma_stream The DMA stream to be enabled
 */
void hal_dma_Enable(hal_dma_Stream_t *dma_stream)
{

    SET_BIT(dma_stream->CR, HAL_DMA_SxCR_EN_MASK);

}

/**
 * @brief Disable the DMA stream
 * @param dma_stream The DMA stream to be disabled
 */
void hal_dma_Disable(hal_dma_Stream_t *dma_stream)
{

    CLEAR_BIT(dma_stream->CR, HAL_DMA_SxCR_EN_MASK);

}

