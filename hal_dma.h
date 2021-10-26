

/**
 * @file    hal_dma.h
 * @brief   STM32F4 DMA peripheral HAL implementation header file
 */


#ifndef __HAL_DMA
#define __HAL_DMA


/************
 * Includes *
 ************/

#include "stm32f4xx.h"


/***********
 * Defines *
 ***********/

/* DMA_SxCR Register Mask/Position */
#define HAL_DMA_SxCR_EN_MASK     DMA_SxCR_EN_Msk
#define HAL_DMA_SxCR_PFCTRL_MASK DMA_SxCR_PFCTRL_Msk
#define HAL_DMA_SxCR_DIR_MASK    DMA_SxCR_DIR_Msk
#define HAL_DMA_SxCR_CIRC_MASK   DMA_SxCR_CIRC_Msk
#define HAL_DMA_SxCR_PINC_MASK   DMA_SxCR_PINC_Msk
#define HAL_DMA_SxCR_MINC_MASK   DMA_SxCR_MINC_Msk
#define HAL_DMA_SxCR_PSIZE_MASK  DMA_SxCR_PSIZE_Msk
#define HAL_DMA_SxCR_MSIZE_MASK  DMA_SxCR_MSIZE_Msk
#define HAL_DMA_SxCR_PINCOS_MASK DMA_SxCR_PINCOS_Msk
#define HAL_DMA_SxCR_PL_MASK     DMA_SxCR_PL_Msk
#define HAL_DMA_SxCR_DBM_MASK    DMA_SxCR_DBM_Msk
#define HAL_DMA_SxCR_CT_MASK     DMA_SxCR_CT_Msk
#define HAL_DMA_SxCR_PBURST_MASK DMA_SxCR_PBURST_Msk
#define HAL_DMA_SxCR_MBURST_MASK DMA_SxCR_MBURST_Msk
#define HAL_DMA_SxCR_CHSEL_MASK  DMA_SxCR_CHSEL_Msk

#define HAL_DMA_SxCR_EN_POSITION     DMA_SxCR_EN_Pos
#define HAL_DMA_SxCR_PFCTRL_POSITION DMA_SxCR_PFCTRL_Pos
#define HAL_DMA_SxCR_DIR_POSITION    DMA_SxCR_DIR_Pos
#define HAL_DMA_SxCR_CIRC_POSITION   DMA_SxCR_CIRC_Pos
#define HAL_DMA_SxCR_PINC_POSITION   DMA_SxCR_PINC_Pos
#define HAL_DMA_SxCR_MINC_POSITION   DMA_SxCR_MINC_Pos
#define HAL_DMA_SxCR_PSIZE_POSITION  DMA_SxCR_PSIZE_Pos
#define HAL_DMA_SxCR_MSIZE_POSITION  DMA_SxCR_MSIZE_Pos
#define HAL_DMA_SxCR_PINCOS_POSITION DMA_SxCR_PINCOS_Pos
#define HAL_DMA_SxCR_PL_POSITION     DMA_SxCR_PL_Pos
#define HAL_DMA_SxCR_DBM_POSITION    DMA_SxCR_DBM_Pos
#define HAL_DMA_SxCR_CT_POSITION     DMA_SxCR_CT_Pos
#define HAL_DMA_SxCR_PBURST_POSITION DMA_SxCR_PBURST_Pos
#define HAL_DMA_SxCR_MBURST_POSITION DMA_SxCR_MBURST_Pos
#define HAL_DMA_SxCR_CHSEL_POSITION  DMA_SxCR_CHSEL_Pos

/* DMA_SxCR_PFCTRL (Peripheral Flow Controller) Enumerations */
#define HAL_DMA_SxCR_PFCTRL_DMA        (0x0)
#define HAL_DMA_SxCR_PFCTRL_PERIPHERAL (0x1)

/* DMA_SxCR_DIR (Data Transfer Direction) Enumerations */
#define HAL_DMA_SxCR_DIR_PERIPHERALTOMEMORY (0x0)
#define HAL_DMA_SxCR_DIR_MEMORYTOPERIPHERAL (0x1)
#define HAL_DMA_SxCR_DIR_MEMORYTOMEMORY     (0x2)

/* DMA_SxCR_PSIZE (Peripheral Data Size) Enumerations */
#define HAL_DMA_SxCR_PSIZE_BYTE     (0x0)
#define HAL_DMA_SxCR_PSIZE_HALFWORD (0x1)
#define HAL_DMA_SxCR_PSIZE_WORD     (0x2)

/* DMA_SxCR_MSIZE (Memory Data Size) Enumerations */
#define HAL_DMA_SxCR_MSIZE_BYTE     (0x0)
#define HAL_DMA_SxCR_MSIZE_HALFWORD (0x1)
#define HAL_DMA_SxCR_MSIZE_WORD     (0x2)

/* DMA_SxCR_PINCOS (Peripheral Increment Offset Size) Enumerations */
#define HAL_DMA_SxCR_PINCOS_PSIZE (0x0)
#define HAL_DMA_SxCR_PINCOS_4     (0x1)

/* DMA_SxCR_PL (Priority Level) Enumerations */
#define HAL_DMA_SxCR_PL_LOW      (0x0)
#define HAL_DMA_SxCR_PL_MED      (0x1)
#define HAL_DMA_SxCR_PL_HIGH     (0x2)
#define HAL_DMA_SxCR_PL_VERYHIGH (0x3)

/* DMA_SxCR_CT (Current Target) Enumerations */
#define HAL_DMA_SxCR_CT_MEMORY0 (0x0)
#define HAL_DMA_SxCR_CT_MEMORY1 (0x1)

/* DMA_SxCR_PBURST (Peripheral Burst) Enumerations */
#define HAL_DMA_SxCR_PBURST_SINGLE (0x0)
#define HAL_DMA_SxCR_PBURST_INCR4  (0x1)-
#define HAL_DMA_SxCR_PBURST_INCR8  (0x2)
#define HAL_DMA_SxCR_PBURST_INCR16 (0x3)

/* DMA_SxCR_MBURST (Memory Burst) Enumerations */
#define HAL_DMA_SxCR_MBURST_SINGLE (0x0)
#define HAL_DMA_SxCR_MBURST_INCR4  (0x1)
#define HAL_DMA_SxCR_MBURST_INCR8  (0x2)
#define HAL_DMA_SxCR_MBURST_INCR16 (0x3)

/* DMA_SxCR_CHSEL (Channel Selection) Enumerations */
#define HAL_DMA_SxCR_CHSEL_CH0 (0x0)
#define HAL_DMA_SxCR_CHSEL_CH1 (0x1)
#define HAL_DMA_SxCR_CHSEL_CH2 (0x2)
#define HAL_DMA_SxCR_CHSEL_CH3 (0x3)
#define HAL_DMA_SxCR_CHSEL_CH4 (0x4)
#define HAL_DMA_SxCR_CHSEL_CH5 (0x5)
#define HAL_DMA_SxCR_CHSEL_CH6 (0x6)
#define HAL_DMA_SxCR_CHSEL_CH7 (0x7)

/* DMA_SxCR_CR Configuration Mask */
#define HAL_DMA_SxCR_CONFIGMASK (HAL_DMA_SxCR_PFCTRL_MASK | \
                                 HAL_DMA_SxCR_DIR_MASK | \
                                 HAL_DMA_SxCR_CIRC_MASK | \
                                 HAL_DMA_SxCR_PINC_MASK | \
                                 HAL_DMA_SxCR_MINC_MASK | \
                                 HAL_DMA_SxCR_PSIZE_MASK | \
                                 HAL_DMA_SxCR_MSIZE_MASK | \
                                 HAL_DMA_SxCR_PINCOS_MASK | \
                                 HAL_DMA_SxCR_PL_MASK | \
                                 HAL_DMA_SxCR_DBM_MASK | \
                                 HAL_DMA_SxCR_CT_MASK | \
                                 HAL_DMA_SxCR_PBURST_MASK | \
                                 HAL_DMA_SxCR_MBURST_MASK | \
                                 HAL_DMA_SxCR_CHSEL_MASK)


/***************************
 * Public Type Definitions *
 ***************************/

/* DMA Port Definition */
typedef DMA_TypeDef hal_dma_Port_t;
typedef DMA_Stream_TypeDef hal_dma_Stream_t;

/* DMA Configuration Parameters */
typedef uint8_t hal_dma_FlowController_t;
typedef uint8_t hal_dma_TransferDirection_t;
typedef uint8_t hal_dma_PeripheralDataSize_t;
typedef uint8_t hal_dma_MemoryDataSize_t;
typedef uint8_t hal_dma_PeripheralIncrementOffsetSize_t;
typedef uint8_t hal_dma_PriorityLevel_t;
typedef uint8_t hal_dma_CurrentTarget_t;
typedef uint8_t hal_dma_PeripheralBurst_t;
typedef uint8_t hal_dma_MemoryBurst_t;
typedef uint8_t hal_dma_Channel_t;

/* DMA Initialization Struct */
typedef struct hal_dma_ConfigStruct_s
{
    hal_dma_FlowController_t                FlowController;                 // Flow control configuration
    hal_dma_TransferDirection_t             TransferDirection;              // Transfer direction
    bool                                    CircularMode;                   // Circular transfer mode
    bool                                    PeripheralIncrementMode;        // Peripheral incrment mode
    bool                                    MemoryIncrementMode;            // Memory increment mode
    hal_dma_PeripheralDataSize_t            PeripheralDataSize;             // Peripheral data transfer size
    hal_dma_MemoryDataSize_t                MemoryDataSize;                 // Memory data transfer size
    hal_dma_PeripheralIncrementOffsetSize_t PeripheralIncrementOffsetSize;  // Peripheral offset increment size mode
    hal_dma_PriorityLevel_t                 PriorityLevel;                  // Stream priority level
    bool                                    DoubleBufferMode;               // Double buffer mode
    hal_dma_CurrentTarget_t                 CurrentTarget;                  // Current target for double buffer mode
    hal_dma_PeripheralBurst_t               PeripheralBurstMode;            // Peripheral burst mode configuration
    hal_dma_MemoryBurst_t                   MemoryBurstMode;                // Memory burst mode configuration
    hal_dma_Channel_t                       ChannelSelection;               // Channel selection
}hal_dma_ConfigStruct_s;


/******************************
 * Public Function Prototypes *
 ******************************/

void hal_dma_Init(hal_dma_Stream_t *dma_stream, hal_dma_ConfigStruct_s *config);
void hal_dma_ConfigureTransfer(hal_dma_Stream_t *dma_stream, uint32_t peripheral_base_address, uint32_t memory_address, uint16_t length);
void hal_dma_Enable(hal_dma_Stream_t *dma_stream);
void hal_dma_Disable(hal_dma_Stream_t *dma_stream);


#endif /* __HAL_DMA */

