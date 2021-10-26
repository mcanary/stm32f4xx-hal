

/**
 * @file    hal_i2s.c
 * @brief   STM32F4 I2S peripheral HAL implementation source file
 */


/************
 * Includes *
 ************/

#include "hal_i2s.h"


/*******************************
 * Private Function Prototypes *
 *******************************/

void hal_i2s_BootstrapDMA(hal_i2s_Port_t *i2s_port);


/********************
 * Public Functions *
 ********************/

/**
 * @brief Configure the specified I2S port
 * @param i2s_port I2S port to be configured
 * @param config   Pointer to the configuration parameter struct
 */
void hal_i2s_Init(hal_i2s_Port_t *i2s_port, hal_i2s_ConfigStruct_s *config)
{

    CLEAR_BIT(i2s_port->I2SCFGR, HAL_I2S_I2SCFGR_I2SE_MASK);

    MODIFY_REG(i2s_port->I2SCFGR, HAL_I2S_I2SCFGR_MASK, ((config->BusMode << HAL_I2S_I2SCFGR_I2SCFG_POSITION) | \
                                                     (config->Standard << HAL_I2S_I2SCFGR_I2SSTD_POSITION) | \
                                                     (config->PCMSync << HAL_I2S_I2SCFGR_PCMSYNC_POSITION) | \
                                                     (config->ClockPolarity << HAL_I2S_I2SCFGR_CKPOL_POSITION) | \
                                                     (config->DataLength << HAL_I2S_I2SCFGR_DATLEN_POSITON) | \
                                                     (config->ChannelLength << HAL_I2S_I2SCFGR_CHLEN_POSITON)));
    MODIFY_REG(i2s_port->I2SPR, HAL_I2S_I2SPR_MASK, ((config->LinearPrescaler << HAL_I2S_I2SPR_I2SDIV_POSITION) | \
                                                 (config->OddPrescalerEnable << HAL_I2S_I2SPR_ODD_POSITION) | \
                                                 (config->MasterClockEnable << HAL_I2S_I2SPR_MCKOE_POSITION)));
    SET_BIT(i2s_port->I2SCFGR, HAL_I2S_I2SCFGR_I2SMOD_MASK);

    SET_BIT(i2s_port->I2SCFGR, HAL_I2S_I2SCFGR_I2SE_MASK);

}

/**
 * @brief Write to the I2S port in a blocking fashion
 * @param i2s_port I2S port to write to
 * @param data     Pointer to the data to be written over the bus
 * @param length   Length of the data to be written
 */
void hal_i2s_WriteData16Blocking(hal_i2s_Port_t *i2s_port, int16_t *data, uint32_t length)
{

    // Assume mono audio
    for(uint32_t index = 0; index < (length * 2); index++)
    {
        WRITE_REG(i2s_port->DR, data[(uint32_t)(index / 2)]);
        while(!READ_BIT(i2s_port->SR, HAL_I2S_SR_TXE_MASK));
    }

}


/**
 * @brief Write to the I2S port in a non-blocking fashion using a DMA
 * @param i2s_port    I2S port to write to
 * @param dma_stream  DMA stream to link to the I2S peripheral for non-blocking write
 * @param dma_channel Channel to use for the specified DMA stream
 * @param data        Pointer to the data to be written over the bus
 * @param length      Length of the data to be written
 */
void hal_i2s_WriteData16NonBlocking(hal_i2s_Port_t *i2s_port, hal_dma_Stream_t *dma_stream, hal_dma_Channel_t dma_channel, int16_t *data, uint32_t length)
{

    hal_dma_ConfigStruct_s dma_config = {
        .FlowController = HAL_DMA_SxCR_PFCTRL_PERIPHERAL,
        .TransferDirection = HAL_DMA_SxCR_DIR_MEMORYTOPERIPHERAL,
        .CircularMode = true,
        .MemoryIncrementMode = true,
        .MemoryDataSize = HAL_DMA_SxCR_MSIZE_HALFWORD,
        .ChannelSelection = dma_channel
    };

    hal_dma_Init(dma_stream, &dma_config);
    hal_dma_ConfigureTransfer(dma_stream, (uint32_t)i2s_port, (uint32_t)data, length);
    hal_dma_Enable(dma_stream);

    hal_i2s_BootstrapDMA(i2s_port);

}

/**
 * @brief Stop an in-progress non-blocking I2S write
 * @param dma_stream The DMA stream being used for the write
 */
void hal_i2s_StopWrite(hal_dma_Stream_t *dma_stream)
{

    hal_dma_Disable(dma_stream);

}


/*********************
 * Private Functions *
 *********************/

/**
 * @brief Bootstrap a configured DMA write for the specified I2S bus
 * @param i2s_port I2S port to write to
 */
void hal_i2s_BootstrapDMA(hal_i2s_Port_t *i2s_port)
{

    WRITE_REG(i2s_port->DR, 0);

}

