/*
  ******************************************************************************
  * Copyright (C) 2025 Adrian P. Nash, G4ZHZ, github/phiphase
  * File:    cw.h
  * Description:
  * Definitions for the CW/side-tone functions
  *****************************************************************************
  */
 #ifndef __CW_H__
 #define __CW_H__

#include "arm_math.h"
#include <math.h>
#ifndef M_PI
#define M_PI 3.1415926535897932384626433832
#endif

typedef struct
{
  float32_t phi;  // Sinewave phase angle
  float32_t dphi; // delta-phi to provide the side-tone frequency
  uint8_t speed;  // CW keying speed
  uint8_t state;  // mark = 1 (keyed), space = 0 (un-keyed)
 } cw_gen_obj;

void init_cw(cw_gen_obj *obj, float32_t fs, float32_t fst);
void gen_cw(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t numSamples);
void gen_dot(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples);
void gen_dash(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples);
void gen_dot_dash(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples);
void gen_dash_dot(cw_gen_obj *obj, q15_t * input, q15_t *output, float32_t amp, uint32_t speed, uint32_t numSamples);
#endif