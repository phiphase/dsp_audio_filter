/********************************************************************************
 * Copyright (C) 2025 Adrian P. Nash, G4ZHZ. All rights reserverd
 * 
 * Inspired by DSP audio filter work at https://github.com/gcallipo/RadioDSP-Pico
 * G. Callipo IK8YFW
 * Adapted for Rasberry pi Pico2 using the Pico SDK 2.1.1 rather than Arduino
 * This DSP audio filter uses DMA transfers and a buffer.
 * 
*********************************************************************************/
#include <stdio.h>
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "pico/stdlib.h"
#include "i2s_out.pio.h"

#define FRAME_LENGTH 128
#define CAPTURE_CHANNEL 0
#define ADC_CLKDIV 63  // Fs = 7.9365 ksps
#define I2S_OUT_DATA_PIN 22
#define I2S_OUT_CLOCK_PIN_BASE 23

// This example uses the default led pin
// You can change this by defining HELLO_PIO_LED_PIN to use a different gpio
#if !defined HELLO_PIO_LED_PIN && defined PICO_DEFAULT_LED_PIN
#define HELLO_PIO_LED_PIN PICO_DEFAULT_LED_PIN
#endif

// The ADC data buffer, capture_buf and
// the DAC output buffer, both 16 bit.
// They are global so that the DMA engines can access them.
uint16_t capture_buf[FRAME_LENGTH];
uint16_t output_buf[FRAME_LENGTH];

int main()
{
    stdio_init_all();

    // Set up the audio output via I2S. A PIO is used for the I2S interface.
    PIO pio;
    uint sm;
    uint offset;

    // This will find a free pio and state machine for our program and load it for us
    // We use pio_claim_free_sm_and_add_program_for_gpio_range so we can address gpios >= 32 if needed and supported by the hardware
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&i2s_out_program, &pio, &sm, &offset, HELLO_PIO_LED_PIN, 1, true);
    hard_assert(success);
    // Configure it to run our program, and start it, using the
    // helper function we included in our .pio file.
    printf("Using gpio %d\n", HELLO_PIO_LED_PIN);
    i2s_out_program_init(pio, sm, offset, HELLO_PIO_LED_PIN);

        // The state machine is now running. Any value we push to its TX FIFO will
    // appear on the LED pin.
    // press a key to exit
    while (getchar_timeout_us(0) == PICO_ERROR_TIMEOUT)
    {
        // Blink
        pio_sm_put_blocking(pio, sm, 1);
        sleep_ms(500);
        // Blonk
        pio_sm_put_blocking(pio, sm, 0);
        sleep_ms(250);
    }

    // This will free resources and unload our program
    pio_remove_program_and_unclaim_sm(&i2s_out_program, pio, sm, offset);
    return (0);
    
    // Set up the ADC
    adc_gpio_init(26 + CAPTURE_CHANNEL);
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

    // Run the DSP filter.
    printf("Starting capture\n");
    adc_run(true);
    
    // Once DMA finishes, stop any new conversions from starting, and clean up
    // the FIFO in case the ADC was still mid-conversion.
    dma_channel_wait_for_finish_blocking(adc_dma_chan);
        printf("Capture finished\n");
    adc_run(false);
    adc_fifo_drain();

    // Print samples to stdout
    for (int i = 0; i < FRAME_LENGTH; ++i) {
        printf("%d\n", capture_buf[i]);
    }
    
}
