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
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "i2s.pio.h"
#include "arm_math.h"
#include "dynamicFilters.h"


#define FRAME_LENGTH 128
#define ADC_RING_BITS 8
#define I2S_RING_BITS 9
#define ADC_GPIO 26 // Pin 31
#define I2S_DATA_GPIO 18  // Pin 24, 25, 26
#define FS 8000  // Sample rate
#define CAPTURE_CHANNEL 0
#define ADC_CLKDIV (48000000/FS)-1 //5999
#define I2S_BIT_RATE 64*FS  // 32 bits


// struct ui_in is used to post User interface updates from the UI to
// the DSP audio filter
struct ui_in
{
    bool filter_on;
    bool nc_on;
    double fl;
    double fh;
    double vol;
};

// struct ui_out is used to post signals from the DSP audio filter to the
// UI such as LEDs.
struct ui_out
{
    bool led;
};

// The ADC data buffer, capture_buf (16 bit) and
// the I2S output buffer (32 bit).
// They are global so that the DMA engines can access them. They are aligned to boundaries
// so that ring buffering can be used.

// ADC capture buffer is 16 bits
int16_t capture_buff[2*FRAME_LENGTH] __attribute__((aligned(sizeof(int16_t)*2*FRAME_LENGTH)));

// I2S output buffer is 32 bits
int32_t output_buff[2*FRAME_LENGTH] __attribute__((aligned(sizeof(int32_t)*2*FRAME_LENGTH)));

// Global so that interrupt handlers have access
uint adc_dma0_chan;
uint adc_dma1_chan;
uint i2s_dma0_chan;
uint i2s_dma1_chan;
uint i2s_semaphore;
uint adc_semaphore;
 
/*
 * global structures that enable the core0 (DSP) to communicate with core1 (UI)
 */
struct ui_in ui_in;   // core 1 (UI) to core 0 (DSP)
struct ui_out ui_out; // core 0 (DSP) to core 1 (UI)
bool ui2dsp_post;  // core 0 reads ui_in
bool dsp2ui_post;  // cpre 1 reads ui_out

/*
 * Critical section variable. Used to get exclusive access to globals used
 * by ISRs
 */
critical_section_t myCS;

/*
 * The DMA interrupt handlers
 */
void dma_isr_0();
void dma_isr_1();

//-----------------------------------------------------------------------------------------------
// Core 1 main entry point                                                                       
// Core 1 handles the user interface and creates the filter coefficients.                                                                   
//-----------------------------------------------------------------------------------------------
void core1_main()
{
 
    const float fs = FS;
    float fc1 = 600;
    float fc2 = 900;
    float b = 80;
    float AdB = 50;

    printf("Core 1 up\n");
    sleep_ms(1000);
    
    
    int NTAPS = kaiserFindN(AdB, b/fs);
    float h[NTAPS];
    printf("NTAPS=%d\n", NTAPS);
    wsfirKBP(h, NTAPS, fc1/fs, fc2/fs, AdB);
/*
    int NTAPS = 64;
    float h[NTAPS];
    wsfirLP(h, NTAPS, W_HANNING, fc2/fs);
*/


    for(int n=0; n < NTAPS; n++)
    {
        printf("%f\n",h[n]);
    }

     printf("Core 1 exiting\n");
    sleep_ms(1000);
    return;
}

//-----------------------------------------------------------------------------------------------
// Core 0 Main entry point                                                                       
//-----------------------------------------------------------------------------------------------
void main()
{
    // PIO for I2S interface
    PIO pio;
    uint sm;
    uint offset;
    bool adc_isr_flag = false;
    uint my_adc_semaphore = 0;
    uint capture_base = 0;
    uint output_base = 0;
    
    critical_section_init(&myCS);
    stdio_init_all();
    setup_default_uart();

    // Initialise the core communications
    dsp2ui_post = false;
    ui2dsp_post = false;

    /*
     * Set up the ADC
     */
    adc_gpio_init(ADC_GPIO + CAPTURE_CHANNEL);
    adc_init();
    adc_select_input(CAPTURE_CHANNEL);
    adc_fifo_setup(
        true,    // Write each completed conversion to the sample FIFO
        true,    // Enable DMA data request (DREQ)
        1,       // DREQ asserted when at least 1 sample present
        false,   // We won't see the ERR bit because of 8 bit reads; disable.
        false
    );
    adc_set_clkdiv(ADC_CLKDIV);

    /*
     * Set up the PIO for the I2S interface.
     * This will find a free pio and state machine for our I2S program and load it for us
     */
    bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&i2s_program, &pio, &sm, &offset, I2S_DATA_GPIO, 1, true);
    hard_assert(success);

    /*
     * Configure the DMAs. Ring buffers are used. Set the ADC buffer write address on a 256-byte boundary and
     * the I2S read address on a 512-byte (I2S) boundary This means that when the buffers start, the DMA pointer LSBs will
     * address the buffer correctly relative to the base address.
     */
    i2s_semaphore = 0;
    adc_semaphore = 0;

    adc_dma0_chan = dma_claim_unused_channel(true);
    adc_dma1_chan = dma_claim_unused_channel(true);
    i2s_dma0_chan = dma_claim_unused_channel(true);
    i2s_dma1_chan = dma_claim_unused_channel(true);
    dma_channel_config adc_dma0_cfg = dma_channel_get_default_config(adc_dma0_chan);
    dma_channel_config adc_dma1_cfg = dma_channel_get_default_config(adc_dma1_chan);
    dma_channel_config i2s_dma0_cfg = dma_channel_get_default_config(i2s_dma0_chan);
    dma_channel_config i2s_dma1_cfg = dma_channel_get_default_config(i2s_dma1_chan);
    channel_config_set_transfer_data_size(&adc_dma0_cfg, DMA_SIZE_16);
    channel_config_set_transfer_data_size(&adc_dma1_cfg, DMA_SIZE_16);
    channel_config_set_transfer_data_size(&i2s_dma0_cfg, DMA_SIZE_32);
    channel_config_set_transfer_data_size(&i2s_dma1_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&adc_dma0_cfg, false);
    channel_config_set_read_increment(&adc_dma1_cfg, false);
    channel_config_set_write_increment(&adc_dma0_cfg, true);
    channel_config_set_write_increment(&adc_dma1_cfg, true);
    channel_config_set_read_increment(&i2s_dma0_cfg, true);
    channel_config_set_read_increment(&i2s_dma1_cfg, true);
    channel_config_set_write_increment(&i2s_dma0_cfg, false);
    channel_config_set_write_increment(&i2s_dma1_cfg, false);
    channel_config_set_dreq(&adc_dma0_cfg, DREQ_ADC);
    channel_config_set_dreq(&adc_dma1_cfg, DREQ_ADC);
    channel_config_set_dreq(&i2s_dma0_cfg, pio_get_dreq(pio, sm, true));
    channel_config_set_dreq(&i2s_dma1_cfg, pio_get_dreq(pio, sm, true));
    channel_config_set_ring(&adc_dma0_cfg, true, ADC_RING_BITS);
    channel_config_set_ring(&adc_dma1_cfg, true, ADC_RING_BITS);
    channel_config_set_ring(&i2s_dma0_cfg, false, I2S_RING_BITS);
    channel_config_set_ring(&i2s_dma1_cfg, false, I2S_RING_BITS);
    dma_channel_configure(
        adc_dma0_chan, &adc_dma0_cfg,
        &capture_buff[0],    // dst
        &adc_hw->fifo,  // src
        FRAME_LENGTH,  // transfer count
        false          // do not start immediately
    );
    dma_channel_configure(
        adc_dma1_chan, &adc_dma1_cfg,
        &capture_buff[FRAME_LENGTH],    // dst
        &adc_hw->fifo,  // src
        FRAME_LENGTH,  // transfer count
        false           // do not start immediately
    );
    dma_channel_configure(
        i2s_dma0_chan, &i2s_dma0_cfg,
        &pio->txf[sm],  // dst
        &output_buff[0],     // src
        FRAME_LENGTH,   // transfer count
        false           // start DMA after the PIO
    );
    dma_channel_configure(
        i2s_dma1_chan, &i2s_dma1_cfg,
        &pio->txf[sm],  // dst
        &output_buff[FRAME_LENGTH],     // src
        FRAME_LENGTH,   // transfer count
        false           // start DMA after the PIO
    );

    /*
     * ADC uses DMA_IRQ_0. I2S uses DMA_IRQ_1
     */
    dma_channel_set_irq0_enabled(adc_dma0_chan, true);
    dma_channel_set_irq0_enabled(adc_dma1_chan, true);
    dma_channel_set_irq1_enabled(i2s_dma0_chan, true);
    dma_channel_set_irq1_enabled(i2s_dma1_chan, true);

    /*
     * Enable the DMA interrupts. Both DMA_IRQ_0 and DMA_IRQ_1 are used.
     */
    irq_set_exclusive_handler(DMA_IRQ_0, dma_isr_0);
    irq_set_enabled(DMA_IRQ_0, true);
    irq_set_exclusive_handler(DMA_IRQ_1, dma_isr_1);
    irq_set_enabled(DMA_IRQ_1, true);

    /* 
     * Fill the output buffer with data. This is to avoid pops and squeaks when
     * we turn on.
     */
    for(uint n=0; n < 2*FRAME_LENGTH; n++)
        output_buff[n] = 0x00000000;

    // Configure PIO to run our pI2S interface, using the
    // helper function we included in our .pio file.
    i2s_program_init(pio, sm, offset, I2S_DATA_GPIO, I2S_BIT_RATE);

    // Start the ADC
    adc_run(true);

    // Start the DMAs
    dma_channel_start(adc_dma0_chan);
    dma_channel_start(i2s_dma0_chan);

    /*
     * Start the other core that deals with the UI
     */
    multicore_launch_core1(core1_main);

    /*-------------------------------------------------------------------------------------------*/
    /* The main processing loop                                                                  */
    /*-------------------------------------------------------------------------------------------*/
    while(1)
    {      
        // Detect an edge on the ADC semaphore to detect an aDC DMA interrupt.
        critical_section_enter_blocking(&myCS);
        if (adc_semaphore != my_adc_semaphore)
        {
            adc_isr_flag = true;
            my_adc_semaphore = adc_semaphore;
        }
        critical_section_exit(&myCS);

        /*
        * If adc_semaphore==1, then assume that capture_buff[0..FRAME_LEN-1] contains
        * un-processed data and that output_buff[0..FRAME_LEN-1] has just been sent to I2S so is free.
        * capture_base = 0, output_base = 0.
        * 
        * If adc_semaphore==0, then assume that capture_buff[FRAME_LEN..2*FRAME_LEN-1] contains
        * un-processed data and that output_buff[FRAME_LEN..2*FRAME_LEN-1] has just been sent to I2S so is free.
        * capture_base = FRAME_LEN, output_base = FRAME_LEN.
        */
        if (adc_isr_flag)
        {
            adc_isr_flag = false;
            
            /*
             * Write the I2S output data. Bits 31:16 are the Right channel.
             * Bits 15:0 are the left channel.
             */
            capture_base = (1-my_adc_semaphore)*FRAME_LENGTH;
            output_base = (1-my_adc_semaphore)*FRAME_LENGTH;

            for(int n=0; n < FRAME_LENGTH; n++)
            {
                int16_t x = capture_buff[capture_base+n] << 3; // 12 to 16 bits
                output_buff[output_base+n] = (int32_t)((x << 16) | x);
                
            }
            //printf("ADC interrupt. capture_base=%u, output_base=%u\n", capture_base, output_base);
        }
 
    }

    /*
     * Cleanup
     */
    adc_run(false);
    adc_fifo_drain();
    dma_channel_cleanup(adc_dma0_chan);
    dma_channel_cleanup(adc_dma1_chan);
    dma_channel_cleanup(i2s_dma0_chan);
    dma_channel_cleanup(i2s_dma1_chan);

}



/*
 * DMA IRQ0 Interrupt Service Routine that handles
 * both ADC DMA channels.
 */
void dma_isr_0()
{        
    if (!adc_semaphore)
    {
        dma_hw->ints0 = (1u << adc_dma0_chan); // Clear interrupt status reg
        adc_semaphore = 1;
        dma_channel_start(adc_dma1_chan);
    }
    else
    {
        dma_hw->ints0 = (1u << adc_dma1_chan); // Clear interrupt status reg
        adc_semaphore = 0;
        dma_channel_start(adc_dma0_chan);
    } 
    
}

/*
 * DMA Interrupt Service Routine that handles
 * both I2S DMA 0 and DMA 1.
 */
void dma_isr_1()
{    
    if (!i2s_semaphore)
    {
        dma_hw->ints1 = (1u << i2s_dma0_chan); // Clear interrupt status register 1, ints1
        dma_channel_start(i2s_dma1_chan);
        i2s_semaphore = 1;
    }
    else
    {
        dma_hw->ints1 = (1u << i2s_dma1_chan);  // Clear interrupt status register 1, ints1
        dma_channel_start(i2s_dma0_chan);
        i2s_semaphore = 0;
    } 
}