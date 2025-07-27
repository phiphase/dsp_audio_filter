/*
  ******************************************************************************
  * Copyright (C) 2025 Adrian P. Nash, G4ZHZ, github/phiphase
  * File:    filter.h
  * Description:
  * Header file for the Kaiser window filter functions used to generate the
  * coefficients for the FIR channel filters.
  * Based on Kaiser Window filter design described in,
  * Openheim A V, Schafer RW, "Discrete-time Signal Processing" 
  * 
  * Notes:
  * The bessi0() function computes the modified Bessel function of the first
  * kind. It is taken from "Numerical Recipes in C"
  * 
  * float_t rather than double is used to halve the memory required and
  * so that the RP2350 microcontroller's FPU can be used.
  * int16_t (16 bit integers) are compatible with 16-bit fixed-point filtering.
  *    
  * Samples from the ADC and to the I2S interface are treated as 16 bit fixed
  * point.
  * 
  * Inspiration taken from Dynamic Calculation of FIR Filters based
  * on Bob Mailing's routines
  *****************************************************************************
  */

#ifndef _FILTER_H_
#define _FILTER_H_

#include "pico/stdlib.h"  // Included so that the int16_t/uint16_t/float32_t32_t types are available.

#define _USE_MATH_DEFINES  
#include <math.h> 

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832
#endif

#define MAX_NTAPS 500

// Function prototypes
void wKaiser(float_t w[], const uint N, float_t alpha, float_t beta);
uint kaiserFindN(float_t AdB, float_t t);
void wsfirKLP(float_t h[], const uint N, float_t fc, float_t AdB);
void wsfirKHP(float_t h[], const uint N, float_t fc, float_t AdB);
void wsfirKBS(float_t h[], const uint N, float_t fc1,  float_t fc2, float_t AdB);
void wsfirKBP(float_t h[], const uint N, float_t fc1,  float_t fc2, float_t AdB);
void genSinc(float_t sinc[], const uint N, float_t fc);
float_t bessi0(float_t x);
#endif