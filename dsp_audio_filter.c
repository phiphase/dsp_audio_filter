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
 * Functions
 * ---------
 * GPIO2 (pin 4) 0: fc/B, 1: fL/fH
 * GPIO3 (pin 5) 0: average filter in, 1: average filter out
 * GPIO4 (pin 6) 0: channel filter in, 1: channel filter out
 * GPIO5 (pin 7) 0: ALE in, 1: ALE out
 * GPIO6 (pin 9) ALE mode: 0: Auto notch, 1: noise reduction
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
#include "cw.h"

//#define PROFILE  // Enable printing of microseconds spent by core 0 doing DSP.
#define ADC0_GPIO 26 // Pin 31
#define FC_AND_B_SEL_GPIO 2   // Take GPIO2 (pin 4) low to enable centre frequency/bandwidth control
#define AVG_FILTER_GPIO 3
#define CH_FILTER_GPIO 4
#define ALE_GPIO 5
#define ALE_MODE_GPIO  6
#define CW_GPIO 7
#define CW_KEYER_DOT_GPIO 8
#define CW_KEYER_DASH_GPIO 9
#define CW_KEYER_OUT_GPIO 10
#define TUNE_GPIO 11
#define LED1_GPIO 12
#define LED2_GPIO 13
#define I2S_DATA_GPIO 18  // Pin 24, 25, 26
#define FRAME_LENGTH 128
#define ADC_RING_BITS 8
#define I2S_RING_BITS 9
#define CAPTURE_CHANNEL 0
#define FS 8000  // Sample rate
#define ADC_CLKDIV (48000000/FS)-1 //5999
#define I2S_BIT_RATE 64*FS  // 32 bits
#define MAX_TAPS 500     // Maximum number of taps of a filter
#define COEFF_SCALE 32768.0
#define SPIN_LOCK_ID 1
#define NAVG 4           // No. averages. must be even and a multiple of 4.
#define NUM_ADC_CH 4     // Number of ADC channels
#define F_MAX 3600.0     // Maximum centre frequency
#define F_MIN 300.0      // Minimum centre frequency
#define B_MAX 3000.0     // Maximum bandwidth
#define B_MIN 100.0      // Minimum bandwidth
#define ADC_MAX 1635     // Maximum ADC output
#define ADC_MIN 2        // Minimum ADC output
#define ADC2_MAX 1631    // Maximum ADC output
#define ADC2_MIN 0       // Minimum ADC output
#define ADC_HYSTERISIS 5 // The amount that the filter corner frequency ADCs have to change by
#define MU 0.07         // LMS algorithm mu
#define NLMS 32         // Number of taps for the LMS filter. Must be a multiple of 4.
#define DELAY 4          // ALE Delay line length
#define MAX_DELAY 32     // Maximum ALE Delay line length
#define NOISE_RED 0      // ALE operates in noise reduction mode
#define AUTO_NOTCH 1     // ALE operates in auto-notch mode
#define NORM_LMS 1
#define ST_PITCH 800     // Side-tone pitch, Hz
#define ST_AMP 0.5      // Amplitude of the side-tone
#define CW_WPM_MAX 40.0
#define CW_WPM_MIN 5.0

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
bool gpio_isr_dot = false;
bool gpio_isr_dash = false;

 
/*
 * Global flags that enable the core-0 (DSP) to communicate with core-1 (UI)
 */
const uint32_t uiAvgFilterIn    = 1;
const uint32_t uiAvgFilterOut   = 2;
const uint32_t uiChFilterIn     = 3;
const uint32_t uiChFilterOut    = 4;
const uint32_t uiChFilterUpdate = 5;
const uint32_t uiALEIn           = 6;
const uint32_t uiALEOut          = 7;
const uint32_t uiALEAutoNotch    = 8;
const uint32_t uiALENoiseRed     = 9;
const uint32_t uiCWOn            = 10;
const uint32_t uiCWOff           = 11;

#ifdef PROFILE
uint32_t time1, time2;
#endif

/*
 * The interrupt handlers and the critical section objects
 */
void dma_isr_0();
void dma_isr_1();
void gpio_isr(uint gpio, uint32_t events);
critical_section_t core0_cs, core1_cs;
spin_lock_t *lock;

/*
 * All in global space so the spin-lock protects
 * AvgFilterObj, avgCoeffs, avgState
 */
arm_fir_instance_q15 AvgFilterObj;
q15_t avgCoeffs[NAVG];
q15_t avgState[NAVG+FRAME_LENGTH];

/* Channel filter
 * All in global space so the spin-lock protects
 * FIRFilterObj, FIRcoeffs, FIRstate
 */
float32_t h[MAX_TAPS];
uint16_t ntaps;
arm_fir_instance_q15 FIRFilterObj;
q15_t FIRcoeffs[MAX_NTAPS];
q15_t FIRstate[MAX_NTAPS+FRAME_LENGTH];

/*
 * Adaptive Line Enhancer (ALE) in global space
 * so that the spinlock protects DelayObj, LMSFilterObj,
 * delayCoeffs, delayState, lmsCoeffs, lmsState
 */
uint32_t numDelayTaps = DELAY;
q15_t delayCoeffs[MAX_DELAY];
q15_t delayState[MAX_DELAY+FRAME_LENGTH];
q15_t lmsCoeffs[NLMS];
q15_t lmsState[NLMS+FRAME_LENGTH];
q15_t mu = (q15_t)(32768.0 * MU);
arm_fir_instance_q15 DelayObj;
arm_lms_norm_instance_q15 LMSFilterObj;
cw_gen_obj STGenObj;

//-----------------------------------------------------------------------------------------------
// CORE-1 MAIN ENTRY POINT                                                                       
// Core 1 handles the user interface and creates the filter coefficients.  It passes commands
// to core-0 when the user interface is changed.                                                                 
//-----------------------------------------------------------------------------------------------
void core1_main()
{
    int16_t adc[NUM_ADC_CH], lastAdc[NUM_ADC_CH];
    bool adcChange[NUM_ADC_CH];  // Flags for ADC value change
    const float32_t fs = FS;    // Hz Sample rate
    float32_t fL = 600;         // Hz lower corner frequency
    float32_t fH = 900;         // Hz upper corner frequency
    float32_t Bt = 80;          // Hz transition bandwidth
    float32_t AdB = 50;         // dB stop-band attenuation
    uint32_t save;              // Used for the spin-lock
    float32_t cwWPM;            // CW Words per minute

#ifdef PROFILE
    double timeAvailable_us = (FRAME_LENGTH/fs)*1.0E6;
#endif

    printf("Core 1 up\n");

    critical_section_init(&core1_cs);
    
    /*
     * Initial channel filter setup
     * Force an initialisation of the filter.
     * h and ntaps are accessed by core0 so need a lock 
     */
    save = spin_lock_blocking(lock);
    ntaps = kaiserFindN(AdB, Bt/fs);   
    wsfirKBP(h, ntaps, fL/fs, fH/fs, AdB); 
    spin_unlock(lock, save);
    multicore_fifo_push_blocking(uiChFilterUpdate);

    /*
     * Set up GPIOs. They are on sequential pins
     */
    for (int i=FC_AND_B_SEL_GPIO; i < (TUNE_GPIO+1); i++)
    {
        gpio_init(i);  
        gpio_set_dir(i, GPIO_IN);  
        gpio_pull_up(i);
    }

    /*
     * Initialise the Keyer GPIO output
     * This is the DC key closure. It should be
     * driven by DMA.
     */
    gpio_init(CW_KEYER_OUT_GPIO);
    gpio_set_dir(CW_KEYER_OUT_GPIO, GPIO_OUT);
    //gpio_pull_up(CW_KEYER_OUT_GPIO);
    gpio_init(LED1_GPIO);
    gpio_set_dir(LED1_GPIO, GPIO_OUT);
    gpio_put(LED1_GPIO, false);
    gpio_init(LED2_GPIO);
    gpio_set_dir(LED2_GPIO, GPIO_OUT);
    gpio_put(LED2_GPIO, false);

    /*
     * Set up the callbacks for the GPIO IRQs
     */
    gpio_set_irq_enabled_with_callback(AVG_FILTER_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true, &gpio_isr);
    gpio_set_irq_enabled(CH_FILTER_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ALE_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ALE_MODE_GPIO, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);    
    gpio_set_irq_enabled(CW_KEYER_DOT_GPIO, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(CW_KEYER_DASH_GPIO, GPIO_IRQ_EDGE_FALL, true);


    /*
     * Initialise the I2C interface and ADS1015 ADC and Zero the last ADC values
     */
    ads1015_init();  
    memset((void *)lastAdc, 0, sizeof(int16_t)*NUM_ADC_CH);
    
    // Set the initial CW keyer speed
    //cwWPM = CW_WPM_MIN + ((float32_t)((float32_t)read_adc(2)-ADC2_MIN)/(float32_t)(ADC2_MAX-ADC2_MIN)) * (float32_t)(CW_WPM_MAX-CW_WPM_MIN); 
    //volume = (float32_t)(read_adc(3)-ADC_MIN)/(float32_t)ADC_MAX;

    /*
     * Set the initial filters in/out according to pin states since the interrupts are edge-triggered
     */
    if(!gpio_get(AVG_FILTER_GPIO))
        multicore_fifo_push_blocking(uiAvgFilterIn);
    else
        multicore_fifo_push_blocking(uiAvgFilterOut);
    if(!gpio_get(CH_FILTER_GPIO))
        multicore_fifo_push_blocking(uiChFilterIn);
    else
        multicore_fifo_push_blocking(uiChFilterOut);
    if(!gpio_get(ALE_GPIO))
        multicore_fifo_push_blocking(uiALEIn);
    else
        multicore_fifo_push_blocking(uiALEOut);
    if(!gpio_get(ALE_MODE_GPIO))
        multicore_fifo_push_blocking(uiALEAutoNotch);
    else
        multicore_fifo_push_blocking(uiALENoiseRed);
    /*
     * MAIN PROCESSING LOOP FOR CORE 1.
     */
    while(1)
    {    
        // ADCs read and see if the ADC value has changed from last time.
        for(int n=0; n < NUM_ADC_CH; n++)
        {
            adcChange[n] = false;
            adc[n] = read_adc(n);  // Centre frequency
            if(abs(adc[n] - lastAdc[n]) > ADC_HYSTERISIS)
                adcChange[n] = true;
            lastAdc[n] = adc[n];
        } 

        /*
         * Deal with ADC 0 (fL) and ADC 1 (fH) changes.
         */
        if(adcChange[0] || adcChange[1])
        {
            printf("ADC change\n");
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
            
            // h and ntaps are accessed by core0 so need a lock
            save = spin_lock_blocking(lock);
            ntaps = kaiserFindN(AdB, Bt/fs);   
            wsfirKBP(h, ntaps, fL/fs, fH/fs, AdB); 
            spin_unlock(lock, save);
            multicore_fifo_push_blocking(uiChFilterUpdate);
        }
        
        /*
         * CW Iambic Electronic keyer
         */
        if(adcChange[2])
            cwWPM = CW_WPM_MIN + ((float32_t)((float32_t)adc[2]-ADC2_MIN)/(float32_t)(ADC2_MAX-ADC2_MIN)) * (float32_t)(CW_WPM_MAX-CW_WPM_MIN); 
        uint32_t TeDot_us = (uint32_t)(60000000.0/(50.0*cwWPM));
        uint32_t TeDash_us = (uint32_t)(180000000.0/(50.0*cwWPM));
        uint32_t TeGap_us = (uint32_t)(60000000.0/(50.0*cwWPM));
        if(gpio_isr_dot)
        {
            multicore_fifo_push_blocking(uiCWOn);            
            gpio_put(CW_KEYER_OUT_GPIO, true);
            gpio_put(LED1_GPIO, true);
            sleep_us(TeDot_us);            
            multicore_fifo_push_blocking(uiCWOff);            
            gpio_put(CW_KEYER_OUT_GPIO, false);
            gpio_put(LED1_GPIO, false);
            sleep_us(TeGap_us);
        }
        gpio_isr_dot = gpio_get(CW_KEYER_DOT_GPIO) ? false : true;
        
        if(gpio_isr_dash)
        {

            multicore_fifo_push_blocking(uiCWOn);            
            gpio_put(CW_KEYER_OUT_GPIO, false);
            gpio_put(LED1_GPIO, false);
            sleep_us(TeDash_us);
            multicore_fifo_push_blocking(uiCWOff);            
            gpio_put(CW_KEYER_OUT_GPIO, true);
            gpio_put(LED1_GPIO, true);
            sleep_us(TeGap_us);
        }
        gpio_isr_dash = gpio_get(CW_KEYER_DASH_GPIO) ? false : true;

        // Straight key
        if(!gpio_get(CW_GPIO))
        {
            
            multicore_fifo_push_blocking(uiCWOn);            
            gpio_put(CW_KEYER_OUT_GPIO, true);
            gpio_put(LED1_GPIO, true);
        }
        else
        {
            multicore_fifo_push_blocking(uiCWOff);            
            gpio_put(CW_KEYER_OUT_GPIO, false);
            gpio_put(LED1_GPIO,false);
        }

        // Tune (no side-tone)
        if(!gpio_get(TUNE_GPIO))
        {
            gpio_put(CW_KEYER_OUT_GPIO, true);
            gpio_put(LED1_GPIO,true);

        }
        else
        {
            gpio_put(CW_KEYER_OUT_GPIO, false);
            gpio_put(LED1_GPIO,false);
        }

        sleep_ms(5);

#ifdef PROFILE
        save = spin_lock_blocking(lock);
        int duration = time2-time1;
        spin_unlock(lock, save);
        if (duration > 0)
            printf("DSP: %d us, utilisation = %1.1f %%\n", duration, 100.0*(double)duration/timeAvailable_us);
#endif

    }    

    printf("Core 1 exiting\n");
    sleep_ms(1000);
    return;
}




//-----------------------------------------------------------------------------------------------
// CORE-0 MAIN ENTRY POINT
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
    uint32_t save;  // Used for the spin-lock

    // Signal path
    q15_t tmp1[FRAME_LENGTH];
    q15_t avgFilterOut[FRAME_LENGTH];
    q15_t chFilterOut[FRAME_LENGTH]; 
    q15_t delayIn[FRAME_LENGTH];
    q15_t delayOut[FRAME_LENGTH];
    q15_t aleOut1[FRAME_LENGTH];
    q15_t aleOut2[FRAME_LENGTH];
    q15_t cwOut[FRAME_LENGTH];
    q15_t *pOut;  // Equal either to tmp1 or tmp2. Used to find the output data
 
    // Flags to control the signal path.
    bool avgFilterIn = false;
    bool chFilterIn = false;
    bool aleIn = false;
    bool aleAutoNotch = false;  // Either NOISE_RED or AUTO_NOTCH
    bool cwOn = false;

    stdio_init_all();
    setup_default_uart();
    lock = spin_lock_init(SPIN_LOCK_ID);
    critical_section_init(&core0_cs);
   
    /*
     * Start the other core that deals with the UI
     */
    multicore_launch_core1(core1_main);

    // Initialise the averaging filter
    for(int i=0; i < NAVG; i++)
        avgCoeffs[i] = (q15_t)(32768.0/(double)NAVG);
    arm_fir_init_q15(&AvgFilterObj, NAVG, avgCoeffs, avgState, FRAME_LENGTH);
    
    // Initialise the ALE delay line and LMS filter          
    arm_fir_init_q15(&DelayObj, numDelayTaps, delayCoeffs, delayState, FRAME_LENGTH);
    for(int i=0 ; i < numDelayTaps; i++)
        delayCoeffs[i] = 0;
    delayCoeffs[0] = 8192;
    for(int i=0; i < NLMS; i++)
        lmsCoeffs[i] = 1;
    arm_lms_norm_init_q15(&LMSFilterObj, NLMS, lmsCoeffs, lmsState, mu, FRAME_LENGTH, 0);
   
    /*
     * Initialise the CW keyer/side-tone
     */
    init_cw(&STGenObj, FS, ST_PITCH);

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
                    avgFilterIn = true;
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
                case uiALEIn:
                    save = spin_lock_blocking(lock);
                    for(int i=0; i < NLMS; i++)
                        lmsCoeffs[i] = 1;
                    arm_lms_norm_init_q15(&LMSFilterObj, NLMS, lmsCoeffs, lmsState, mu, FRAME_LENGTH, 0);
                    spin_unlock(lock, save);
                    aleIn = true;
                    break;
                case uiALEOut:
                    aleIn = false;
                    break;
                case uiALEAutoNotch:
                    aleAutoNotch = true;
                    break;
                case uiALENoiseRed:
                    aleAutoNotch = false;
                    break;
                case uiChFilterUpdate:                
                    save = spin_lock_blocking(lock);
                    for (int i=0; i < ntaps; i++)
                        FIRcoeffs[ntaps-i-1] = (q15_t)(h[i]*COEFF_SCALE);
                    arm_fir_init_q15(&FIRFilterObj, ntaps, FIRcoeffs, FIRstate, FRAME_LENGTH);
                    spin_unlock(lock, save);
                    break;
                case uiCWOn:
                    cwOn = true;
                    break;
                case uiCWOff:
                    cwOn = false;
                    break;
   
                // Invalid command - ignore
                default:
                    break;
            }
        }

        // Detect an edge on the ADC semaphore to detect an aDC DMA interrupt.
        critical_section_enter_blocking(&core0_cs);
        if (adc_semaphore != my_adc_semaphore)
        {
            adc_isr_flag = true;
            my_adc_semaphore = adc_semaphore;
        }
        critical_section_exit(&core0_cs);

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
#ifdef PROFILE            
            save = spin_lock_blocking(lock);
            time1 = time_us_32();
            spin_unlock(lock, save);
#endif            
            adc_isr_flag = false;            
            capture_base = (1-my_adc_semaphore)*FRAME_LENGTH;
            output_base = (1-my_adc_semaphore)*FRAME_LENGTH;

            /*
             * Shift the 12-bit input signal to 16 bits (1.15 format).
             */
            for(int i=0; i < FRAME_LENGTH; i++)
                tmp1[i] = capture_buff[capture_base+i] << 3;

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

                // Automatic Line Enhancer (ALE)
                if (aleIn)
                {
                    arm_fir_q15(&DelayObj, pOut, delayOut, FRAME_LENGTH); 
                    arm_lms_norm_q15(&LMSFilterObj, delayOut, pOut, aleOut1, aleOut2, FRAME_LENGTH);                    
                    pOut = aleAutoNotch ? aleOut2 : aleOut1;
                }

                // CW Side-tone    
                if(cwOn)            
                {
                    gen_cw(&STGenObj, pOut, cwOut, ST_AMP, FRAME_LENGTH);
                    pOut = cwOut;
                }

            /*
             * Output the signal to the I2S buffer
             */
            
            for(int i=0; i < FRAME_LENGTH; i++)
                output_buff[output_base+i] = (int32_t)((pOut[i] << 16) | pOut[i]);
#ifdef PROFILE
            save = spin_lock_blocking(lock);
            time2 = time_us_32();
            spin_unlock(lock, save);
#endif            
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

/*
 * GPIO ISR Core-1)
 */
void gpio_isr(uint gpio, uint32_t events)
{

    switch(gpio)
    {
        case AVG_FILTER_GPIO:
            if (!gpio_get(AVG_FILTER_GPIO) && (events & GPIO_IRQ_EDGE_FALL))
                multicore_fifo_push_blocking(uiAvgFilterIn);
            if (gpio_get(AVG_FILTER_GPIO) && (events & GPIO_IRQ_EDGE_RISE))
                multicore_fifo_push_blocking(uiAvgFilterOut);                
            break;
        case CH_FILTER_GPIO:
            if (!gpio_get(CH_FILTER_GPIO) && (events & GPIO_IRQ_EDGE_FALL))
                multicore_fifo_push_blocking(uiChFilterIn);
            if (gpio_get(CH_FILTER_GPIO) && (events & GPIO_IRQ_EDGE_RISE))
                multicore_fifo_push_blocking(uiChFilterOut);
            break;
        case ALE_GPIO:                      
            if (!gpio_get(ALE_GPIO) && (events & GPIO_IRQ_EDGE_FALL))
                multicore_fifo_push_blocking(uiALEIn);
            if (gpio_get(ALE_GPIO) && (events & GPIO_IRQ_EDGE_RISE))
                multicore_fifo_push_blocking(uiALEOut);
            break;
        case ALE_MODE_GPIO:                      
            if (!gpio_get(ALE_GPIO) && (events & GPIO_IRQ_EDGE_FALL))
                multicore_fifo_push_blocking(uiALEAutoNotch);
            if (gpio_get(ALE_GPIO) && (events & GPIO_IRQ_EDGE_RISE))
                multicore_fifo_push_blocking(uiALENoiseRed);
            break; 
        case CW_KEYER_DOT_GPIO:
            if (events & GPIO_IRQ_EDGE_FALL)
                gpio_isr_dot = true;          
            break;
        case CW_KEYER_DASH_GPIO:
            if (events & GPIO_IRQ_EDGE_FALL)
                gpio_isr_dash = true;          
            break;
 
        default:
            printf("un-supported GPIO\n");
            break;
    }
}