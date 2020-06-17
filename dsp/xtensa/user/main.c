/*
 * Copyright (c) 2018-2019 NXP USA, Inc.
 * All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <xtensa/config/core.h>
#include <xtensa/xos.h>

#include "fsl_common.h"
#include "fsl_inputmux.h"
#include "pin_mux.h"

#include "fsl_debug_console.h"
#include "fsl_dma.h"
#include "fsl_dmic.h"
#include "fsl_dmic_dma.h"
#include "fsl_i2s.h"
#include "fsl_i2s_dma.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#ifdef RT600_VALIDATION_BOARD
#define BOARD_XTAL_SYS_CLK_HZ 16000000U /*!< Board xtal_sys frequency in Hz */
#else
#define BOARD_XTAL_SYS_CLK_HZ 24000000U /*!< Board xtal_sys frequency in Hz */
#endif
#define BOARD_XTAL32K_CLK_HZ 32768U /*!< Board xtal32K frequency in Hz */

#define DEMO_DMA (DMA1)

#define DEMO_I2S_MASTER_CLOCK_FREQUENCY CLOCK_GetMclkClkFreq()
#define DEMO_I2S_TX (I2S4)
#define DEMO_I2S_TX_CHANNEL (9)
#define DEMO_I2S_SAMPLE_RATE 48000
#define I2S_CLOCK_DIVIDER (24576000 / DEMO_I2S_SAMPLE_RATE / 32 / 8)
/* I2S source clock 24.576MHZ, sample rate 48KHZ, bits width 32, 8 channel, \
so bitclock should be 48KHZ * 32 = 1563KHZ, divider should be 24.576MHZ / 1563KHZ */

#define DEMO_DMA_CHANNEL0 0U
#define DMA_DESCRIPTOR_RINGBUF_NUM 3U

#define DEMO_DMA_CHANNEL0 0U
#define DEMO_DMA_CHANNEL1 1U
#define DMA_DESCRIPTOR_RINGBUF_NUM 3U

#define DEMO_DMIC_DMA_RX_CHANNEL_0 16U
#define DEMO_DMIC_DMA_RX_CHANNEL_1 17U
#define DEMO_DMIC_DMA_RX_CHANNEL_2 18U
#define DEMO_DMIC_DMA_RX_CHANNEL_3 19U
#define DEMO_DMIC_DMA_RX_CHANNEL_4 20U
#define DEMO_DMIC_DMA_RX_CHANNEL_5 21U
#define DEMO_DMIC_DMA_RX_CHANNEL_6 22U
#define DEMO_DMIC_DMA_RX_CHANNEL_7 23U

#define DEMO_DMIC_CHANNEL_0 kDMIC_Channel0
#define DEMO_DMIC_CHANNEL_1 kDMIC_Channel1
#define DEMO_DMIC_CHANNEL_2 kDMIC_Channel2
#define DEMO_DMIC_CHANNEL_3 kDMIC_Channel3
#define DEMO_DMIC_CHANNEL_4 kDMIC_Channel4
#define DEMO_DMIC_CHANNEL_5 kDMIC_Channel5
#define DEMO_DMIC_CHANNEL_6 kDMIC_Channel6
#define DEMO_DMIC_CHANNEL_7 kDMIC_Channel7

#define DMIC_NUMS (8U)

#define FIFO_DEPTH (15U)
#define BUFFER_SIZE (256)
#define BUFFER_NUM (2U)
#define RING_EightCHANNEL_BUFFER_SIZE (2 * BUFFER_NUM * BUFFER_SIZE)

#define STACK_SIZE (XOS_STACK_MIN_SIZE + 0x1024)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/

static void BOARD_InitClock(void);
static void Disable_Cache(void);

static void DMA_Config(void);
static void DEMO_DMICChannelConfigurations(void);
static void DEMO_DMA_Memory_to_Momery_Configurations_Channel0_3(void);
static void DEMO_DMA_Memory_to_Momery_Configurations_Channel4_7(void);
static void DEMO_I2SConfigurations(void);

int32_t thread_AudioTask_func(void *arg, int32_t unused);
int32_t thread_func(void *arg, int32_t unused);
extern void DMA_IRQHandle(DMA_Type *base);
extern int NonCacheable_start, NonCacheable_end;
extern int NonCacheable_init_start, NonCacheable_init_end;

/*******************************************************************************
 * Variables
 ******************************************************************************/
DMA_ALLOCATE_LINK_DESCRIPTORS(s_dma_table_0, DMA_DESCRIPTOR_RINGBUF_NUM * BUFFER_NUM);
DMA_ALLOCATE_LINK_DESCRIPTORS(s_dma_table_1, DMA_DESCRIPTOR_RINGBUF_NUM * BUFFER_NUM);

static i2s_config_t tx_config;
static i2s_transfer_t i2sTxTransfer;

//DMA SRC BUFFER
DMA_ALLOCATE_DATA_TRANSFER_BUFFER(uint8_t s_buffer_0[BUFFER_SIZE * BUFFER_NUM], sizeof(uint8_t));
DMA_ALLOCATE_DATA_TRANSFER_BUFFER(uint8_t s_buffer_1[BUFFER_SIZE * BUFFER_NUM], sizeof(uint8_t));
//DMA DES BUFFER
DMA_ALLOCATE_DATA_TRANSFER_BUFFER(uint8_t s_destBuffer[RING_EightCHANNEL_BUFFER_SIZE * DMA_DESCRIPTOR_RINGBUF_NUM], sizeof(uint8_t));
static uint32_t volatile I2S_tx_start_0 = 0;
static uint32_t volatile I2S_tx_start_1 = 0;
static uint32_t volatile I2S_tx_ring_state = 0;
static uint32_t volatile MTOM_StartProcess = 0;
/* DMIC dma handle for 8 channel */
static dmic_dma_handle_t s_dmicDmaHandle[DMIC_NUMS];
/* dma handle for 8 channel */
static dma_handle_t s_dmaHandle[DMIC_NUMS];
/* i2s dma handle */
static dma_handle_t s_i2sTxDmaHandle;
static i2s_dma_handle_t s_i2sTxHandle;
/* memory to memory dma handle */
static dma_handle_t s_dma_DMICProcessHandle_0;
static dma_handle_t s_dma_DMICProcessHandle_1;
static dma_channel_config_t transferConfig_0;
static dma_channel_config_t transferConfig_1;
/* ping pong descriptor */
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_0[2], 16);
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_1[2], 16);
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_2[2], 16);
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_3[2], 16);
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_4[2], 16);
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_5[2], 16);
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_6[2], 16);
SDK_ALIGN(dma_descriptor_t s_dmaDescriptorPingpong_7[2], 16);
/* dmic transfer configurations */
static dmic_transfer_t s_receiveXfer_0[2U] = {
    /* transfer configurations for channel0 */
    {
        .data                   = s_buffer_0,
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_0[1],
    },

    {
        .data                   = &s_buffer_0[BUFFER_SIZE],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_0[0],
    },
};
static dmic_transfer_t s_receiveXfer_1[2U] = {
    /* transfer configurations for channel1 */
    {
        .data                   = &s_buffer_0[2],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_1[1],
    },

    {
        .data                   = &s_buffer_0[BUFFER_SIZE+2],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_1[0],
    },
};
static dmic_transfer_t s_receiveXfer_2[2U] = {
    /* transfer configurations for channel1 */
    {
        .data                   = &s_buffer_0[4],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_2[1],
    },

    {
        .data                   = &s_buffer_0[BUFFER_SIZE+4],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_2[0],
    },
};
static dmic_transfer_t s_receiveXfer_3[2U] = {
    /* transfer configurations for channel1 */
    {
        .data                   = &s_buffer_0[6],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_3[1],
    },

    {
        .data                   = &s_buffer_0[BUFFER_SIZE+6],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_3[0],
    },
};
static dmic_transfer_t s_receiveXfer_4[2U] = {
    /* transfer configurations for channel0 */
    {
        .data                   = s_buffer_1,
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_4[1],
    },

    {
        .data                   = &s_buffer_1[BUFFER_SIZE],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_4[0],
    },
};
static dmic_transfer_t s_receiveXfer_5[2U] = {
    /* transfer configurations for channel1 */
    {
        .data                   = &s_buffer_1[2],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_5[1],
    },

    {
        .data                   = &s_buffer_1[BUFFER_SIZE+2],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_5[0],
    },
};
static dmic_transfer_t s_receiveXfer_6[2U] = {
    /* transfer configurations for channel1 */
    {
        .data                   = &s_buffer_1[4],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_6[1],
    },

    {
        .data                   = &s_buffer_1[BUFFER_SIZE+4],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_6[0],
    },
};
static dmic_transfer_t s_receiveXfer_7[2U] = {
    /* transfer configurations for channel1 */
    {
        .data                   = &s_buffer_1[6],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_7[1],
    },

    {
        .data                   = &s_buffer_1[BUFFER_SIZE+6],
        .dataWidth              = sizeof(uint16_t),
        .dataSize               = BUFFER_SIZE/4,
        .dataAddrInterleaveSize = kDMA_AddressInterleave4xWidth,
        .linkTransfer           = &s_receiveXfer_7[0],
    },
};
/* dmic channel configurations */
static dmic_channel_config_t s_dmicChannelConfig = {
    .divhfclk            = kDMIC_PdmDiv1,
    .osr                 = 32U,
    .gainshft            = 3U,
    .preac2coef          = kDMIC_CompValueZero,
    .preac4coef          = kDMIC_CompValueZero,
    .dc_cut_level        = kDMIC_DcCut155,
    .post_dc_gain_reduce = 1U,
    .saturate16bit       = 1U,
    .sample_rate         = kDMIC_PhyFullSpeed,
    .enableSignExtend    = false,
};

XosThread thread_Audio_tcb;
static uint8_t thread_Audio_stack[STACK_SIZE];

/*******************************************************************************
 * Code
 ******************************************************************************/

void dmic_Callback(DMIC_Type *base, dmic_dma_handle_t *handle, status_t status, void *userData)
{
    uint8_t *dmic_dma_channel = userData;
    if(*dmic_dma_channel == DEMO_DMIC_DMA_RX_CHANNEL_7)
    {
        MTOM_StartProcess = 1;
    }
}

void DMIC0_3_U16CovU32_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    I2S_tx_start_0 = 1;
}

void DMIC4_7_U16CovU32_Callback(dma_handle_t *handle, void *param, bool transferDone, uint32_t tcds)
{
    I2S_tx_start_1 = 1;
}
/*
"C:\Program Files (x86)\Tensilica\Xtensa OCD Daemon 14.01\xt-ocd" -dTD=80 -l log0.txt
 */
int main()
{
	int32_t ret;
	Disable_Cache();
	BOARD_Init_UserPin();
	BOARD_InitClock();
	BOARD_InitDebugConsole();
	// Set clock frequency before calling xos_start_main().
	xos_set_clock_freq(XOS_CLOCK_FREQ);
	// Select and start system timer.
	xos_start_system_timer(-1, 0);

	xos_start_main("main", 5, 0);
	// Create a thread after control returns.
	ret = xos_thread_create(&thread_Audio_tcb,
							0,
							thread_AudioTask_func,
							0,
							"Audio",
							thread_Audio_stack,
							STACK_SIZE,
							7,
							0,
							0);
	while(1);
	return 0;
}

int32_t thread_AudioTask_func(void *arg, int32_t unused)
{
    DMA_Config();
    DEMO_I2SConfigurations();
    DEMO_DMICChannelConfigurations();
    DEMO_DMA_Memory_to_Momery_Configurations_Channel0_3();
    DEMO_DMA_Memory_to_Momery_Configurations_Channel4_7();

    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[0], s_receiveXfer_0, DEMO_DMIC_CHANNEL_0);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[1], s_receiveXfer_1, DEMO_DMIC_CHANNEL_1);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[2], s_receiveXfer_2, DEMO_DMIC_CHANNEL_2);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[3], s_receiveXfer_3, DEMO_DMIC_CHANNEL_3);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[4], s_receiveXfer_4, DEMO_DMIC_CHANNEL_4);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[5], s_receiveXfer_5, DEMO_DMIC_CHANNEL_5);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[6], s_receiveXfer_6, DEMO_DMIC_CHANNEL_6);
    DMIC_TransferReceiveDMA(DMIC0, &s_dmicDmaHandle[7], s_receiveXfer_7, DEMO_DMIC_CHANNEL_7);

    DMA_SubmitChannelDescriptor(&s_dma_DMICProcessHandle_0, &(s_dma_table_0[1]));
    DMA_StartTransfer(&s_dma_DMICProcessHandle_0);
    DMA_SubmitChannelDescriptor(&s_dma_DMICProcessHandle_1, &(s_dma_table_1[1]));
    DMA_StartTransfer(&s_dma_DMICProcessHandle_1);

    PRINTF("Start Playback. \r\n");

    I2S_tx_ring_state = 0;

    //continuous transmit two buffer firstly
    i2sTxTransfer.data     = s_destBuffer + I2S_tx_ring_state * RING_EightCHANNEL_BUFFER_SIZE;
    i2sTxTransfer.dataSize = RING_EightCHANNEL_BUFFER_SIZE;
    while(I2S_TxTransferSendDMA(DEMO_I2S_TX, &s_i2sTxHandle, i2sTxTransfer) != kStatus_Success);
    I2S_tx_ring_state = (I2S_tx_ring_state + 1) % DMA_DESCRIPTOR_RINGBUF_NUM;

    i2sTxTransfer.data     = s_destBuffer + I2S_tx_ring_state * RING_EightCHANNEL_BUFFER_SIZE;
    i2sTxTransfer.dataSize = RING_EightCHANNEL_BUFFER_SIZE;
    while(I2S_TxTransferSendDMA(DEMO_I2S_TX, &s_i2sTxHandle, i2sTxTransfer) != kStatus_Success);
    I2S_tx_ring_state = (I2S_tx_ring_state + 1) % DMA_DESCRIPTOR_RINGBUF_NUM;

    while (1)
    {
        if(MTOM_StartProcess == 1)
        {
           MTOM_StartProcess = 0;
           DMA_SubmitChannelTransfer(&s_dma_DMICProcessHandle_0, &transferConfig_0);
           DMA_StartTransfer(&s_dma_DMICProcessHandle_0);
           DMA_SubmitChannelTransfer(&s_dma_DMICProcessHandle_1, &transferConfig_1);
           DMA_StartTransfer(&s_dma_DMICProcessHandle_1);
        }
        if (I2S_tx_start_0 == 1 && I2S_tx_start_1 == 1)
        {
            I2S_tx_start_0 = 0;
            I2S_tx_start_1 = 0;
            i2sTxTransfer.data     = s_destBuffer + I2S_tx_ring_state * RING_EightCHANNEL_BUFFER_SIZE;
            i2sTxTransfer.dataSize = RING_EightCHANNEL_BUFFER_SIZE;
            while(I2S_TxTransferSendDMA(DEMO_I2S_TX, &s_i2sTxHandle, i2sTxTransfer) != kStatus_Success);
            I2S_tx_ring_state = (I2S_tx_ring_state + 1) % DMA_DESCRIPTOR_RINGBUF_NUM;
        }
    }
	return 0;
}

static void BOARD_InitClock(void)
{
	/* sets external XTAL OSC freq */
    CLOCK_SetXtalFreq(BOARD_XTAL_SYS_CLK_HZ);

    CLOCK_EnableClock(kCLOCK_InputMux);

	/* attach AUDIO PLL clock to FLEXCOMM4 (I2S4) */
    CLOCK_AttachClk(kAUDIO_PLL_to_FLEXCOMM4);

	/* attach AUDIO PLL clock to MCLK */
	CLOCK_AttachClk(kAUDIO_PLL_to_MCLK_CLK);
	CLOCK_SetClkDiv(kCLOCK_DivMclkClk, 2);
	SYSCTL1->MCLKPINDIR = SYSCTL1_MCLKPINDIR_MCLKPINDIR_MASK;

    /* DMIC source from audio pll, divider 8, 24.576M/8=3.072MHZ */
    CLOCK_AttachClk(kAUDIO_PLL_to_DMIC_CLK);
    CLOCK_SetClkDiv(kCLOCK_DivDmicClk, 8);
}

static void DMA_Config(void)
{
    uint32_t i = 0;

    /* DMA configurations */
    DMA_Init(DEMO_DMA);

	INPUTMUX_AttachSignal(INPUTMUX, 18U, kINPUTMUX_Dmac1ToDspInterrupt); /* XCHAL_EXTINT19_NUM, intlevel 2 */
	xos_register_interrupt_handler(XCHAL_EXTINT19_NUM,(XosIntFunc *) DMA_IRQHandle, DEMO_DMA);
	xos_interrupt_enable(XCHAL_EXTINT19_NUM);

    for (i = 0; i < DMIC_NUMS; i++)
    {
        /* configurations for DMIC channel */
        DMA_EnableChannel(DEMO_DMA, i + DEMO_DMIC_DMA_RX_CHANNEL_0);
        DMA_SetChannelPriority(DEMO_DMA, DEMO_DMIC_DMA_RX_CHANNEL_0 + i, kDMA_ChannelPriority1);
        DMA_CreateHandle(&s_dmaHandle[i], DEMO_DMA, DEMO_DMIC_DMA_RX_CHANNEL_0 + i);
    }

    /* configurations for I2S channel */
    DMA_EnableChannel(DEMO_DMA, DEMO_I2S_TX_CHANNEL);
    DMA_SetChannelPriority(DEMO_DMA, DEMO_I2S_TX_CHANNEL, kDMA_ChannelPriority3);
    DMA_CreateHandle(&s_i2sTxDmaHandle, DEMO_DMA, DEMO_I2S_TX_CHANNEL);
}
static void Disable_Cache(void)
{
    /* Disable DSP cache for noncacheable sections. */
	xthal_set_region_attribute((void *) 0x0, 0x20000000,XCHAL_CA_BYPASS, XTHAL_CAFLAG_EXACT);
	xthal_set_region_attribute((void *) 0x20000000, 0x40000000,XCHAL_CA_BYPASS, XTHAL_CAFLAG_EXACT);
}

static void DEMO_DMICChannelConfigurations(void)
{
	PRINTF("Configure DMIC0_7 Channel \r\n");
    /* dmic channel configurations */
    DMIC_Init(DMIC0);
    DMIC_Use2fs(DMIC0, true);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)0, (stereo_side_t)false, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 0, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[0], dmic_Callback, &s_dmaHandle[0].channel, &s_dmaHandle[0]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[0], &s_dmaDescriptorPingpong_0,   2);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)1, (stereo_side_t)true, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 1, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[1], dmic_Callback, &s_dmaHandle[1].channel, &s_dmaHandle[1]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[1], &s_dmaDescriptorPingpong_1,   2);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)2, (stereo_side_t)false, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 2, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[2], dmic_Callback, &s_dmaHandle[2].channel, &s_dmaHandle[2]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[2], &s_dmaDescriptorPingpong_2,   2);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)3, (stereo_side_t)true, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 3, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[3], dmic_Callback, &s_dmaHandle[3].channel, &s_dmaHandle[3]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[3], &s_dmaDescriptorPingpong_3,   2);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)4, (stereo_side_t)false, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 4, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[4], dmic_Callback, &s_dmaHandle[4].channel, &s_dmaHandle[4]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[4], &s_dmaDescriptorPingpong_4,   2);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)5, (stereo_side_t)true, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 5, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[5], dmic_Callback, &s_dmaHandle[5].channel, &s_dmaHandle[5]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[5], &s_dmaDescriptorPingpong_5,   2);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)6, (stereo_side_t)false, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 6, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[6], dmic_Callback, &s_dmaHandle[6].channel, &s_dmaHandle[6]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[6], &s_dmaDescriptorPingpong_6,   6);

    DMIC_ConfigChannel(DMIC0, (dmic_channel_t)7, (stereo_side_t)true, &s_dmicChannelConfig);
    DMIC_FifoChannel(DMIC0, 7, FIFO_DEPTH, true, true);
    DMIC_TransferCreateHandleDMA(DMIC0, &s_dmicDmaHandle[7], dmic_Callback, &s_dmaHandle[7].channel, &s_dmaHandle[7]);
    DMIC_InstallDMADescriptorMemory(&s_dmicDmaHandle[7], &s_dmaDescriptorPingpong_7,   2);

    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH0(1));
    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH1(1));
    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH2(1));
    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH3(1));
    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH4(1));
    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH5(1));
    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH6(1));
    DMIC_EnableChannnel(DMIC0, DMIC_CHANEN_EN_CH7(1));
}

static void DEMO_DMA_Memory_to_Momery_Configurations_Channel0_3(void)
{
	PRINTF("Configure DMIC0_3 DMA\r\n");
    DMA_CreateHandle(&s_dma_DMICProcessHandle_0, DEMO_DMA, DEMO_DMA_CHANNEL0);
    DMA_SetChannelPriority(DEMO_DMA, DEMO_DMA_CHANNEL0, kDMA_ChannelPriority0);
    DMA_EnableChannel(DEMO_DMA, DEMO_DMA_CHANNEL0);
    DMA_SetCallback(&s_dma_DMICProcessHandle_0, DMIC0_3_U16CovU32_Callback, NULL);

    DMA_SetupDescriptor(&(s_dma_table_0[0]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_1,
                        &s_destBuffer[6],
                        &(s_dma_table_0[1]));

    DMA_SetupDescriptor(&(s_dma_table_0[1]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_1 + BUFFER_SIZE,
                        &s_destBuffer[6 + RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_0[2]));

    DMA_SetupDescriptor(&(s_dma_table_0[2]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_1,
                        &s_destBuffer[6 + 2 * RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_0[3]));

    DMA_SetupDescriptor(&(s_dma_table_0[3]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_1 + BUFFER_SIZE,
                        &s_destBuffer[6],
                        &(s_dma_table_0[4]));

    DMA_SetupDescriptor(&(s_dma_table_0[4]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_1,
                        &s_destBuffer[6 + RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_0[5]));

    DMA_SetupDescriptor(&(s_dma_table_0[5]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_1 + BUFFER_SIZE,
                        &s_destBuffer[6 + 2 * RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_0[0]));
}

static void DEMO_DMA_Memory_to_Momery_Configurations_Channel4_7(void)
{
	PRINTF("Configure DMIC4_7 DMA\r\n");
    DMA_CreateHandle(&s_dma_DMICProcessHandle_1, DEMO_DMA, DEMO_DMA_CHANNEL1);
    DMA_SetChannelPriority(DEMO_DMA, DEMO_DMA_CHANNEL1, kDMA_ChannelPriority0);
    DMA_EnableChannel(DEMO_DMA, DEMO_DMA_CHANNEL1);
    DMA_SetCallback(&s_dma_DMICProcessHandle_1, DMIC4_7_U16CovU32_Callback, NULL);

    DMA_SetupDescriptor(&(s_dma_table_1[0]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_0,
                        &s_destBuffer[2],
                        &(s_dma_table_1[1]));

    DMA_SetupDescriptor(&(s_dma_table_1[1]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_0 + BUFFER_SIZE,
                        &s_destBuffer[2 + RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_1[2]));

    DMA_SetupDescriptor(&(s_dma_table_1[2]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_0,
                        &s_destBuffer[2 + 2 * RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_1[3]));

    DMA_SetupDescriptor(&(s_dma_table_1[3]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_0 + BUFFER_SIZE,
                        &s_destBuffer[2],
                        &(s_dma_table_1[4]));

    DMA_SetupDescriptor(&(s_dma_table_1[4]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_0,
                        &s_destBuffer[2 + RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_1[5]));

    DMA_SetupDescriptor(&(s_dma_table_1[5]),
                        DMA_CHANNEL_XFER(true, true, true, false, 2U, kDMA_AddressInterleave1xWidth,
                        kDMA_AddressInterleave4xWidth, BUFFER_SIZE),
                        s_buffer_0 + BUFFER_SIZE,
                        &s_destBuffer[2 + 2 * RING_EightCHANNEL_BUFFER_SIZE],
                        &(s_dma_table_1[0]));
}

static void DEMO_I2SConfigurations(void)
{
    PRINTF("Configure I2S\r\n");
     /*
     * masterSlave = kI2S_MasterSlaveNormalMaster;
     * mode = kI2S_ModeI2sClassic;
     * rightLow = false;
     * leftJust = false;
     * pdmData = false;
     * sckPol = false;
     * wsPol = false;
     * divider = 1;
     * oneChannel = false;
     * dataLength = 16;
     * frameLength = 32;
     * position = 0;
     * fifoLevel = 4;
     */
    I2S_TxGetDefaultConfig(&tx_config);
    tx_config.divider     = I2S_CLOCK_DIVIDER;
    tx_config.mode        = kI2S_ModeDspWsShort;
    tx_config.wsPol       = true;
    tx_config.dataLength  = 32U;
    tx_config.frameLength = 32 * 4 * 2;
    I2S_TxInit(DEMO_I2S_TX, &tx_config);
    I2S_EnableSecondaryChannel(DEMO_I2S_TX, kI2S_SecondaryChannel1, false, 64);
    I2S_EnableSecondaryChannel(DEMO_I2S_TX, kI2S_SecondaryChannel2, false, 128);
    I2S_EnableSecondaryChannel(DEMO_I2S_TX, kI2S_SecondaryChannel3, false, 192);
    I2S_TxTransferCreateHandleDMA(DEMO_I2S_TX, &s_i2sTxHandle, &s_i2sTxDmaHandle, NULL, NULL);
}
