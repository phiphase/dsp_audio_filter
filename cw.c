/*
 ******************************************************************************
 * Copyright (C) 2025 Adrian P. Nash, G4ZHZ, github/phiphase
 * File:    cw.c
 * Description:
 * Implements the CW keyer and side-tone functions
 *****************************************************************************
 */

#include "cw.h"

// Type I Bi-quad IIR filter section (2nd order) fc = 800 Hz
// Coefficients calculated by https://www.earlevel.com/main/2021/09/02/biquad-calculator-v3/
const float32_t zeros[3] = {0.06745508395870334, 0.13491016791740668, 0.06745508395870334};
const float32_t poles[2] = {-1.1429772843080923, 0.41279762014290533};


/*
 * init_cw() Initialise the CW generator
 */
void init_cw(cw_gen_obj *obj, float32_t fs, float32_t fst)
{
    obj->dphi = 2.0*M_PI* (fst/fs);
    obj->n = 0;

  // Initialise the filter
#ifdef Q15
  obj->coeffs[0] = (q15_t)(zeros[0] * 16384.0);
  obj->coeffs[1] = 0;
  obj->coeffs[2] = (q15_t)(zeros[1] * 16384.0);
  obj->coeffs[3] = (q15_t)(zeros[2] * 16384.0);
  obj->coeffs[4] = (q15_t)(poles[0] * 16384.0);
  obj->coeffs[5] = (q15_t)(poles[1] * 16384.0);
  arm_biquad_cascade_df1_init_q15 (&obj->filterObj, 1,&obj->coeffs[0],  &obj->state[0], 0);
#else
  obj->coeffs[0] = zeros[0];
  obj->coeffs[1] = zeros[1];
  obj->coeffs[2] = zeros[2];
  obj->coeffs[3] = poles[0];
  obj->coeffs[4] = poles[1];
  arm_biquad_cascade_df1_init_f32	(&obj->filterObj, 1, &obj->coeffs[0], &obj->state[0]); 	
#endif
  for(int i=0; i < 4; i++)
    obj->state[i] = 0;
    return;
}


void gen_cw(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t numSamples)
{
#ifdef Q15  
    q15_t st_in[128];
    q15_t st_out[128];
    for(int i=0; i < numSamples; i++)
      st_in[i] = (q15_t)(amp * arm_sin_f32(obj->dphi*obj->n++) * 32768.0);
    arm_biquad_cascade_df1_q15(&obj->filterObj, st_in, st_out, numSamples);
    arm_add_q15(st_out, input, output, numSamples);
#else  
  float32_t st_in[128];
  float32_t st_out[128];
  for(int i=0; i < numSamples; i++)
      st_in[i] = amp * arm_sin_f32(obj->dphi*obj->n++);
  arm_biquad_cascade_df1_f32(&obj->filterObj, st_in, st_out, numSamples);
  for(int i=0; i < numSamples; i++)
    output[i] = (q15_t)(st_out[i]*32768.0) + input[i];
#endif



}
