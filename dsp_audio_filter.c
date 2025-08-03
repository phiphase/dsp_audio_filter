/********************************************************************************
 * Copyright (C) 2025 Adrian P. Nash, G4ZHZ. All rights reserverd
 * DSP Audio filter
 * The input audio is to the Pico2's built-in ADC. The output audio is via I2S
 * to an external I2S DAC
 * DMA is used to transfer samples from the ADC to the capture buffer, and from
 * the output buffer to the  I2S which is implemented in a PIO.
 * Core 0 does the DSP signal path. Core 1 does the user interface and filter
 * coefficient generation.
 * 
 * Note: float32_t is an arm_math type. The standart C/C++ definition is float
 * doubles are used for non-time-critical variables that are non-integer.
 * 
*********************************************************************************/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "hardware/clocks.h"
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include "pico/critical_section.h"
#include "pico/multicore.h"
#include "i2s.pio.h"
#include "arm_math.h"
#include "filter.h"
#include "ads1015.h"

#define ADC0_GPIO 26 // Pin 31
#define FC_AND_B_SEL_GPIO 2   // Take GPIO2 (pin 4) low to enable centre frequency/bandwidth control
#define I2S_DATA_GPIO 18  // Pin 24, 25, 26


#define FRAME_LENGTH 128
#define ADC_RING_BITS 8
#define I2S_RING_BITS 9
#define CAPTURE_CHANNEL 0
#define FS 8000  // Sample rate
#define ADC_CLKDIV (48000000/FS)-1 //5999
#define I2S_BIT_RATE 64*FS  // 32 bits
#define MAX_TAPS 500  // Maximum number of taps of a filter
#define SPIN_LOCK_ID 1
#define NAVG 4  // No. averages. must be even and a multiple of 4.
#define NLMS 64  // Number of taps for the LMS filter. Must be a multiple of 4.
#define NOISE_RED 0      // NC operates in noise reduction mode
#define AUTO_NOTCH 1     // NC operates in auto-notch mode
#define NUM_ADC_CH 4     // Number of ADC channels
#define F_MAX 3600.0  // Maximum centre frequency
#define F_MIN 300.0   // Minimum centre frequency
#define B_MAX 3000.0 // Maximum bandwidth
#define B_MIN 100.0   // Minimum bandwidth
#define ADC_MAX 1635     // Maximum ADC output
#define ADC_MIN 2        // Minimum ADC output
#define ADC_HYSTERISIS 5 // The amount that the ADC value has to change by

/*
 * The ADC data buffer, capture_buf (16 bit) and
 * the I2S output buffer (32 bit).
 * They are global so that the DMA engines can access them. They are aligned to boundaries
 * so that ring buffering can be used.
 */
int16_t capture_buff[2*FRAME_LENGTH] __attribute__((aligned(sizeof(int16_t)*2*FRAME_LENGTH)));
int32_t output_buff[2*FRAME_LENGTH] __attribute__((aligned(sizeof(int32_t)*2*FRAME_LENGTH)));

/*
 * Global so that interrupt handlers have access
 */
uint adc_dma0_chan;
uint adc_dma1_chan;
uint i2s_dma0_chan;
uint i2s_dma1_chan;
uint i2s_semaphore;
uint adc_semaphore;
 
/*
 * Global structures that enable the core-0 (DSP) to communicate with core-1 (UI)
 */
const uint32_t uiAvgFilterIn    = 1;
const uint32_t uiAvgFilterOut   = 2;
const uint32_t uiChFilterIn     = 3;
const uint32_t uiChFilterOut    = 4;
const uint32_t uiChFilterUpdate = 5;
const uint32_t uiNCIn           = 6;
const uint32_t uiNCOut          = 7;
const uint32_t uiNCAutoNotch    = 8;
const uint32_t uiNCNoiseRed     = 9;

spin_lock_t *lock;
float_t h[MAX_TAPS];
uint16_t ntaps;

uint32_t time1, time2;

/*
 * The DMA interrupt handlers and the critical section object
 */
void dma_isr_0();
void dma_isr_1();
critical_section_t myCS;

//-----------------------------------------------------------------------------------------------
// Core 1 main entry point                                                                       
// Core 1 handles the user interface and creates the filter coefficients.  It passes commands
// to core-0 when the user interface is changed.                                                                 
//-----------------------------------------------------------------------------------------------
void core1_main()
{
    int16_t adc[NUM_ADC_CH], lastAdc[NUM_ADC_CH];
    bool adcChange[NUM_ADC_CH];  // Flags for ADC value change
    const float_t fs = FS;       // Hz Sample rate
    float_t fL = 600;            // Hz lower corner frequency
    float_t fH = 900;            // Hz upper corner frequency
    float_t Bt = 80;             // Hz transition bandwidth
    float_t AdB = 50;            // dB stop-band attenuation
    
    printf("Core 1 up\n");
    
    // Set up GPIOs
    gpio_init(FC_AND_B_SEL_GPIO);
    gpio_set_dir(FC_AND_B_SEL_GPIO, GPIO_IN);
	gpio_pull_up(FC_AND_B_SEL_GPIO);

    // Initialise the I2C interface and ADS1015 ADC
    ads1015_init();
    
    // Zero the last ADC values
    memset((void *)lastAdc, 0, sizeof(int16_t)*NUM_ADC_CH);

    while(1)
    {
        // ADC reads and see if the ADC value has changed from last time.
        for(int n=0; n < NUM_ADC_CH; n++)
        {
            adc[n] = read_adc(n);  // Centre frequency
            adcChange[n] = false;
            if(abs(adc[n] - lastAdc[n]) > ADC_HYSTERISIS)
                adcChange[n] = true;
            lastAdc[n] = adc[n];
        }     
        
        if(adcChange[0] || adcChange[1])
        {
            if (gpio_get(FC_AND_B_SEL_GPIO)==0)
            {
                float fc = F_MIN + ((float)(adc[0]-ADC_MIN)/(float)(ADC_MAX-ADC_MIN)) * (F_MAX- F_MIN);
                float bw = B_MIN + ((float)(adc[1]-ADC_MIN)/(float)(ADC_MAX-ADC_MIN)) * (float)(B_MAX-B_MIN);
                fL = fc - bw/2;
                fH = fc + bw/2;
            }
            else
            {

                fL = F_MIN + ((float)(adc[0]-ADC_MIN)/(float)(ADC_MAX-ADC_MIN)) * (float)(F_MAX-F_MIN);
                fH = F_MIN + ((float)(adc[1]-ADC_MIN)/(float)(ADC_MAX-ADC_MIN)) * (float)(F_MAX-F_MIN);
            }
            // Put limits on fL and fH.
            fL = fL < F_MIN ? F_MIN:fL;
            fL = fL > F_MAX ? F_MAX:fL;
            fH = fH < F_MIN ? F_MIN:fH;
            fH = fH > F_MAX ? F_MAX:fH;
            
            float_t local_h[MAX_TAPS];
            uint16_t local_ntaps = kaiserFindN(AdB, Bt/fs);   
            wsfirKBP(local_h, local_ntaps, fL/fs, fH/fs, AdB); 

            // h and ntaps are accessed by core0 so need a lock
            uint32_t save = spin_lock_blocking(lock);
            ntaps = local_ntaps;
            memcpy((void *)h, (void *)local_h, ntaps*sizeof(float_t));
            spin_unlock(lock, save);

            multicore_fifo_push_blocking(uiChFilterUpdate);
        }

        multicore_fifo_push_blocking(uiAvgFilterOut);
        multicore_fifo_push_blocking(uiChFilterIn);
        multicore_fifo_push_blocking(uiNCOut);
        multicore_fifo_push_blocking(uiNCAutoNotch);

/*
    // For MATLAB
    printf("ntaps=%u\n", ntaps);
    for(uint i=0; i < ntaps; i++)
        printf("%f\n",h[i]);
*/      sleep_ms(100);

    }
    
    /*while(1)
    {
        sleep_ms(100);
        uint32_t save = spin_lock_blocking(lock);
        int duration = time2-time1;
        if (duration > 0)
            printf("duration = %d us\n", (int)duration);
        spin_unlock(lock, save);
    }*/

    printf("Core 1 exiting\n");
    sleep_ms(1000);
    return;
}

//-----------------------------------------------------------------------------------------------
// Core 0 Main entry point
// Core 0 handles the signal path                                                                     
//-----------------------------------------------------------------------------------------------
void main()
{
    PIO pio;
    uint sm;
    uint offset;
    bool adc_isr_flag = false;
    uint my_adc_semaphore = 0;
    uint capture_base = 0;
    uint output_base = 0;
    
    // Averaging filter
    q15_t avgCoeffs[NAVG];
    q15_t avgState[NAVG+FRAME_LENGTH-1];
    arm_fir_instance_q15 AvgFilterObj;

    // Channel filter
    arm_fir_instance_q15 FIRFilterObj;
    q15_t FIRcoeffs[MAX_NTAPS];
    q15_t FIRstate[MAX_NTAPS];

    // LMS Filter
    arm_lms_norm_instance_q15 LMSFilterObj;
    q15_t lmsCoeffs[NLMS];
    q15_t lmsState[NLMS + FRAME_LENGTH-1];
    q15_t mu = (q15_t)(32768.0 * 0.001);

    q15_t dlyCoeffs[4];
    q15_t dlyState[4+FRAME_LENGTH-1];
    arm_fir_instance_q15 DlyFilterObj;


    // Signal path
    q15_t tmp1[FRAME_LENGTH];
    q15_t tmp2[FRAME_LENGTH];
    q15_t tmp3[FRAME_LENGTH];
    q15_t avgFilterOut[FRAME_LENGTH];
    q15_t chFilterOut[FRAME_LENGTH];
    q15_t ncOut[FRAME_LENGTH];
    q15_t *pOut;  // Equal either to tmp1 or tmp2. Used to find the output data

    // Flags to control the signal path. Default is
    // straight through, i.e. no filtering
    bool chFilterIn = false;
    bool avgFilterIn = false;
    bool ncIn = false;
    int ncMode = NOISE_RED;  // Either NOISE_RED (false) or AUTO_NOTCH (true)

    stdio_init_all();
    setup_default_uart();
    lock = spin_lock_init(SPIN_LOCK_ID);
    critical_section_init(&myCS);
   
    // initialise the averaging filter
    for(int i=0; i < NAVG; i++)
        avgCoeffs[i] = (q15_t)(32768.0/(double)NAVG);
    arm_fir_init_q15(&AvgFilterObj, NAVG, avgCoeffs, avgState, FRAME_LENGTH);

    // Initialise the LMS filter
    for(int i=0; i < NLMS; i++)
        lmsCoeffs[i] = (q15_t)(32768.0*0.5);
    arm_lms_norm_init_q15(&LMSFilterObj, NLMS, lmsCoeffs, lmsState, mu, FRAME_LENGTH, 0);		
    /*for(int i=0; i < 4; i++)
        dlyCoeffs[i]=0;
    dlyCoeffs[3] = (q15_t)32767;
    arm_fir_init_q15(&DlyFilterObj, 4, dlyCoeffs, dlyState, FRAME_LENGTH);
*/
    /*
     * Start the other core that deals with the UI
     */
    multicore_launch_core1(core1_main);

    /*
     * Set up the audio ADC
     */
    
    adc_gpio_init(ADC0_GPIO);
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

    /*-------------------------------------------------------------------------------------------*/
    /* The main processing loop                                                                  */
    /*-------------------------------------------------------------------------------------------*/
    while(1)
    {    
        /*
         * Check for commands from core-1.
         */
        if (multicore_fifo_rvalid())
        {   
            switch(multicore_fifo_pop_blocking())
            {
                case uiAvgFilterIn:
                    avgFilterIn = false;
                    break;
                case uiAvgFilterOut:
                    avgFilterIn = false;
                    break;
                case uiChFilterIn:
                    chFilterIn = true;
                    break;
                case uiChFilterOut:
                    chFilterIn = false;
                    break;
                case uiNCIn:
                    ncIn = true;
                    break;
                case uiNCOut:
                    ncIn = false;
                    break;
                case uiNCAutoNotch:
                    ncMode = AUTO_NOTCH;
                    break;
                case uiNCNoiseRed:
                    ncMode = NOISE_RED;
                    break;
                case uiChFilterUpdate:
                    //uint32_t save = spin_lock_blocking(lock);
                    for (int i=0; i < ntaps; i++)
                        FIRcoeffs[ntaps-i-1] = (q15_t)(h[i]*32768.0);
                    //spin_unlock(lock, save);
                    arm_fir_init_q15(&FIRFilterObj, ntaps, FIRcoeffs, FIRstate, FRAME_LENGTH);
                    break;
                // Invalid command - ignore
                default:
                    break;
            }
        }

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
            time1 = time_us_32();
            adc_isr_flag = false;
            
            capture_base = (1-my_adc_semaphore)*FRAME_LENGTH;
            output_base = (1-my_adc_semaphore)*FRAME_LENGTH;

            /*
             * Shift the 12-bit input signal to 16 bits (1.15 format).
             */
            for(int i=0; i < FRAME_LENGTH; i++)
                tmp1[i] = capture_buff[capture_base+i] << 3; // 12 to 16 bits   
            
                // Averaging Filter
                if (avgFilterIn)                
                {
                    arm_fir_q15(&AvgFilterObj, tmp1, avgFilterOut, FRAME_LENGTH);
                    pOut = avgFilterOut;
                }
                else
                    pOut = tmp1;

                // Channel Filter
                if (chFilterIn)
                {                    
                    arm_fir_q15(&FIRFilterObj, pOut, chFilterOut, FRAME_LENGTH);
                    pOut = chFilterOut;
                }

                // Noise Cancellation
                if (ncIn)
                {
                    //arm_fir_q15(&DlyFilterObj, pOut, dly, FRAME_LENGTH);
                    //arm_lms_norm_q15(&LMSFilterObj, pIn, nCOut, tmp2, tmp3, FRAME_LENGTH);
                    //if (ncMode == NOISE_RED)                
                    //    pOut = tmp2;
                    //else
                    //    pOut = tmp3;                        
                    //}	 	        
                }
                    
            
            /*
             * Output the signal to the I2S buffer
             */
            for(int i=0; i < FRAME_LENGTH; i++)
                output_buff[output_base+i] = (int32_t)((pOut[i] << 16) | pOut[i]);
        
        /*
            // Straight-through connection
            for(int n=0; n < FRAME_LENGTH; n++)
            {
                int16_t x = capture_buff[capture_base+n] << 3; // 12 to 16 bits
                output_buff[output_base+n] = (int32_t)((x << 16) | x);
            
            }
        */
            time2 = time_us_32();
        }




    }

    /*
     * Cleanup
     */
    adc_run(false);
    adc_fifo_drain();
    multicore_fifo_drain();
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