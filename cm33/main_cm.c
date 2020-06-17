/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_debug_console.h"

#include "board.h"
#include "clock_config.h"
#include "pin_mux.h"
#include "fsl_power.h"
#include "fsl_pca9420.h"
#include "fsl_codec_common.h"
#include "fsl_codec_adapter.h"
#include "fsl_cs42888.h"
#include "fsl_i2c.h"
#include "dsp_config.h"
#include "dsp_support.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_CODEC_RESET_PORT 1U
#define DEMO_CODEC_RESET_PIN 9U
#define DEMO_CODEC_I2C_BASEADDR I2C2
#define DEMO_CODEC_I2C_INSTANCE 2U
#define DEMO_CODEC_I2C_CLOCK_FREQ CLOCK_GetFlexCommClkFreq(2U)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void DEMO_InitCS42888(void);
extern void BORAD_CodecReset(bool state);
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* cs42888 configurations */
cs42888_config_t cs42888Config = {
    .DACMode      = kCS42888_ModeSlave,
    .ADCMode      = kCS42888_ModeSlave,
    .reset        = BORAD_CodecReset,
    .master       = false,
    .i2cConfig    = {.codecI2CInstance = DEMO_CODEC_I2C_INSTANCE, .codecI2CSourceClock = 99000000U},
    .format       = {.mclk_HZ = 12288000U, .sampleRate = 48000U, .bitWidth = 24U},
    .bus          = kCS42888_BusTDM,
    .slaveAddress = CS42888_I2C_ADDR,
};
codec_config_t boardCodecConfig = {.codecDevType = kCODEC_CS42888, .codecDevConfig = &cs42888Config};
extern codec_config_t boardCodecConfig;
uint8_t codecHandleBuffer[] = {0U};
codec_handle_t *codecHandle                  = (codec_handle_t *)codecHandleBuffer;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*!
 * @brief Main function
 */
int main(void)
{
    /* Initialize standard SDK demo application pins */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\n8-Channel DMIC Acquire and i2s playback demo--Cortex-M33.\r\n");
    
    CLOCK_EnableClock(kCLOCK_InputMux);

    /* attach FRO clock to FLEXCOMM2(I2C2) */
    CLOCK_AttachClk(kFFRO_to_FLEXCOMM2);
    
    /* init CS42888 */
    DEMO_InitCS42888();
    
    /* Clear MUA reset before run DSP core */
    RESET_PeripheralReset(kMU_RST_SHIFT_RSTn);

    /* Print the initial banner */
    PRINTF("DSP start.\r\n");

    /* Copy DSP image to RAM and start DSP core. */
    BOARD_DSP_Init();

    while (1)
        ;
}

/*******************************************************************************
 * Code
 ******************************************************************************/

static void DEMO_InitCS42888(void)
{
    /*CS42888 RESET PIN*/
    gpio_pin_config_t gpio_config = {
        kGPIO_DigitalOutput,
        0,
    };
    
    PRINTF("Init CS42888 codec\r\n");
    
    GPIO_PortInit(GPIO, DEMO_CODEC_RESET_PORT);
    GPIO_PinInit(GPIO, DEMO_CODEC_RESET_PORT, DEMO_CODEC_RESET_PIN, &gpio_config);

    if (CODEC_Init(codecHandle, &boardCodecConfig) != kStatus_Success)
    {
        PRINTF("CODEC_Init failed!\r\n");
    }
    else
    {
        PRINTF("CODEC_Init Success!\r\n");
    }
}

void BORAD_CodecReset(bool state)
{
    if (state)
    {
        GPIO_PortSet(GPIO, DEMO_CODEC_RESET_PORT, DEMO_CODEC_RESET_PORT << DEMO_CODEC_RESET_PIN);
    }
    else
    {
        GPIO_PortClear(GPIO, DEMO_CODEC_RESET_PORT, DEMO_CODEC_RESET_PORT << DEMO_CODEC_RESET_PIN);
    }
}