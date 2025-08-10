/*
  ******************************************************************************
  * Copyright (C) 2025 Adrian P. Nash, G4ZHZ, github/phiphase
  * File:    cw.c
  * Description:
  * Implements the CW keyer and side-tone functions
  *****************************************************************************
  */

#include "cw.h"

/*
 * init_cw() Initialise the CW generator
 */
void init_cw(cw_gen_obj *obj, float32_t fs, float32_t fst)
{
    obj->state = 0;    
    obj->phi = 0.0;  // Sinewave phase angle
    obj->dphi = 2.0*M_PI* (fst/fs);
    obj->speed = 12;  // CW keying speed
    return;
 }


void gen_cw(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t numSamples)
{
    for(int i=0; i < numSamples; i++)
    {
        output[i] = (q15_t)(amp * sin(obj->phi) * 32768.0) + input[i];
        obj->phi += obj->dphi;
    }
    return;
}

void gen_dot(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples)
{
    return;
}

void gen_dash(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples)
{
    return;
}

void geno_dot_dash(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples)
{
    return;
}

void gen_dash_dot(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples)
{
    return;
}
