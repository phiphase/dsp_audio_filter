/********************************************************************************
 * Copyright (C) 2025 Adrian P. Nash, G4ZHZ. All rights reserverd
 * DSP Audio filter
 * The input audio is to the Pico2's built-in ADC. The output audio is via I2S
 * to an external I2S DAC
 * DMA is used to transfer samples from the ADC to the capture buffer, and from
 * the output buffer to the  I2S which is implemented in a PIO.
 * 
*********************************************************************************/
#include <stdio.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "i2s.pio.h"

#define FRAME_LENGTH 128
#define ADC_RING_BITS 8
#define I2S_RING_BITS 9
#define ADC_GPIO 26
#define CAPTURE_CHANNEL 0
#define ADC_CLKDIV 63  // Fs = 7.9365 ksps
#define I2S_DATA_GPIO 18  // Pin 24, 25, 26
#define I2S_BIT_RATE 64*8000  // 512 kHz - 32 bits, 8000sps

// The ADC data buffer, capture_buf and
// the DAC output buffer, both 16 bit.
// They are global so that the DMA engines can access them. They are aligned to boundaries
// so that ring buffering can be used
int16_t capture_buf[FRAME_LENGTH] __attribute__((aligned(sizeof(int16_t)*FRAME_LENGTH)));
int32_t output_buf[FRAME_LENGTH] __attribute__((aligned(sizeof(int32_t)*FRAME_LENGTH)));

// Global so that interrupt handlers have access
uint adc_dma_chan;
uint i2s_dma_chan;

// The DMA interrupt handler.
void dma_isr();

int main()
{
    // PIO for I2S interface
    PIO pio;
    uint sm;
    uint offset;

    stdio_init_all();
    setup_default_uart();

    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range so we can address gpios >= 32 if needed and supported by the hardware
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&i2s_program, &pio, &sm, &offset, I2S_DATA_GPIO, 1, true);
    hard_assert(success);

    // Configure the I2S DMA
    i2s_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config i2s_dma_cfg = dma_channel_get_default_config(i2s_dma_chan);
    channel_config_set_transfer_data_size(&i2s_dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&i2s_dma_cfg, true);
    channel_config_set_write_increment(&i2s_dma_cfg, false);
    channel_config_set_dreq(&i2s_dma_cfg, pio_get_dreq(pio, sm, true));

    // Set the buffer read address on a 512-byte boundary
    channel_config_set_ring(&i2s_dma_cfg, false, I2S_RING_BITS);
    dma_channel_configure(
        i2s_dma_chan, &i2s_dma_cfg,
        &pio->txf[sm],  // dst
        output_buf,     // src
        FRAME_LENGTH,   // transfer count
        false           // start DMA after the PIO
    );

    dma_channel_set_irq0_enabled(i2s_dma_chan, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_isr);
    irq_set_enabled(DMA_IRQ_0, true);

    // Fill the output buffer with data
    for(int n=0; n < FRAME_LENGTH; n++)
    {
        output_buf[n] = 0x5555AAAA;
    }

    // Configure PIO to run our program, and start it, using the
    // helper function we included in our .pio file.
    printf("I2S data using GPIO %d\n", I2S_DATA_GPIO);
    printf("I2S Clocks using GPIOs %d (BCLK) and %d (LRCLK)\n", I2S_DATA_GPIO+1, I2S_DATA_GPIO+2);
    i2s_program_init(pio, sm, offset, I2S_DATA_GPIO, I2S_BIT_RATE);

    // Now run the I2S DMA.
    dma_channel_start(i2s_dma_chan);
    printf("I2S DMA channel started\n");

    int idx = 0;
    while(1)
    {
        // Do nothing
    }
    
/*
    // Set up the ADC
    adc_gpio_init(ADC_GPIO + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ (and IRQ) asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false
    );
    adc_set_clkdiv(ADC_CLKDIV);
    // Set up the DMA to start transferring data as soon as data appears in FIFO
    // Set up the DMA to start transferring data as soon as data appears in FIFO
    uint adc_dma_chan = dma_claim_unused_channel(true);
    dma_channel_config adc_dma_cfg = dma_channel_get_default_config(adc_dma_chan);

    // Reading from constant address, writing to incrementing int16 addresses
    channel_config_set_transfer_data_size(&adc_dma_cfg, DMA_SIZE_16);
    channel_config_set_read_increment(&adc_dma_cfg, false);
    channel_config_set_write_increment(&adc_dma_cfg, true);

    // Pace transfers based on availability of ADC samples
    channel_config_set_dreq(&adc_dma_cfg, DREQ_ADC);

    dma_channel_configure(
        adc_dma_chan, &adc_dma_cfg,
        capture_buf,    // dst
        &adc_hw->fifo,  // src
        FRAME_LENGTH,  // transfer count
        true           // start immediately
    );

    void dma_irh() {
    //printf("Interrupt\n");

    // Re-start the DMA
    dma_channel_start(i2s_dma_chan);
    // Clear interrupt for trigger DMA channel.
    dma_hw->ints0 = (1u << i2s_dma_chan);
}
*/

    dma_channel_cleanup(adc_dma_chan);
    dma_channel_cleanup(i2s_dma_chan);

}

/*
 * DMA Interrupt Service Routine
 */
void dma_isr()
{
    
    // Re-start the DMA
    dma_channel_start(i2s_dma_chan);
    // Clear interrupt for trigger DMA channel.
    dma_hw->ints0 = (1u << i2s_dma_chan);
}