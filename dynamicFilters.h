/*
  ******************************************************************************
  * File    dynamicFilters.h
  * Author  Adrian Nash, G4ZHZ
  * from Giuseppe Callipo's Github project SimpleAudioFilter09 ik8yfw@libero.it
  * Description:
  *    Dynamic Calculation of FIR Filters based on Bob Mailing's routines
  *    This file is part of RadioDSP MINI project.
  *    Converted doubles to floats so that a FPU can be used.
  *
  *****************************************************************************
  */

/*
  Windowed Sinc FIR Generator
  Bob Maling (BobM.DSP@gmail.com)
  Contributed to musicdsp.org Source Code Archive
  Last Updated: April 12, 2005

  http://www.musicdsp.org/showone.php?id=194

  Usage:
    Lowpass:  wsfirLP(h, N, WINDOW, fc)
    Highpass: wsfirHP(h, N, WINDOW, fc)
    Bandpass: wsfirBP(h, N, WINDOW, fc1, fc2)
    Bandstop: wsfirBS(h, N, WINDOW, fc1, fc2)

  where:
    h (float[N]):  filter coefficients will be written to this array
    N (int):    number of taps
    WINDOW (int): Window (W_BLACKMAN, W_HANNINGING, or W_HAMMING)
    fc (float):  cutoff (0 < fc < 0.5, fc = f/fs)
            --> for fs=48kHz and cutoff f=12kHz, fc = 12k/48k => 0.25

    fc1 (float): low cutoff (0 < fc < 0.5, fc = f/fs)
    fc2 (float): high cutoff (0 < fc < 0.5, fc = f/fs)


  Windows included here are Blackman, Hanning, and Hamming. Other windows can be
  added by doing the following:
    1. "Window type constants" section: Define a global constant for the new window.
    2. wsfirLP() function: Add the new window as a case in the switch() statement.
    3. Create the function for the window

       For windows with a design parameter, such as Kaiser, some modification
       will be needed for each function in order to pass the parameter.
*/
#ifndef _WSFIR_H_
#define _WSFIR_H_

#include "pico/stdlib.h"  // Included so that the int16_t/uint16_t/float32_t types are available.

#define _USE_MATH_DEFINES  
#include <math.h> 

#ifndef M_PI
#define M_PI 3.1415926535897932384626433832
#endif
#define NUM_COEFFICIENTS 64

// ID's for the different types of filters
#define ID_LOWPASS   1
#define ID_HIGHPASS  2
#define ID_BANDPASS  3
#define ID_BANDSTOP  4


// ID's for window type contstants
static const int W_BLACKMAN =  1;
static const int W_HANNING  =  2;
static const int W_HAMMING  =  3;

// Function prototypes
void audioFilter(float h[], const int N, const int TYPE, const int WINDOW,  float fc1,  float fc2, float fc);
void bandpass(float h[], const int N, const int WINDOW, float fc1,  float fc2);
void wsfirLP(float h[], const int N, const int WINDOW, float fc);
void wsfirHP(float h[], const int N, const int WINDOW, float fc);
void wsfirBS(float h[], const int N, const int WINDOW, float fc1,  float fc2);
void wsfirBP(float h[], const int N, const int WINDOW,  float fc1, float fc2);
void genSinc(float sinc[], const int N, float fc);
void wBlackman(float w[], const int N);
void wHanning(float w[], const int N);
void wHamming(float w[], const int N);

void convertCoeffToInt16(float in[], int16_t out[], const int N);

// Added Kaiser window filter functions.
float bessi0(float x);
void wKaiser(float w[], const int N, float alpha, float beta);
int kaiserFindN(float AdB, float t);
void wsfirKLP(float h[], const int N, float fc, float AdB);
void wsfirKHP(float h[], const int N, float fc, float AdB);
void wsfirKBS(float h[], const int N, float fc1,  float fc2, float AdB);
void wsfirKBP(float h[], const int N, float fc1,  float fc2, float AdB);



#endif