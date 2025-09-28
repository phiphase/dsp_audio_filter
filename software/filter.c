/*
  ******************************************************************************
  * Copyright (C) 2025 Adrian P. Nash, G4ZHZ, github/phiphase
  * File:    filter.c
  * Description:
  * Implementation of the Kaiser window filter functions used to generate the
  * coefficients for the FIR channel filters.
  * Based on Kaiser Window filter design described in,
  * Openheim A V, Schafer RW, "Discrete-time Signal Processing" 
  * 
  * Notes:
  * The bessi0() function computes the modified Bessel function of the first
  * kind. It is taken from "Numerical Recipes in C"
  * 
  * float32_t32_t is used so that the RP2350 microcontroller's FPU can be used.
  * int16_t (16 bit integers) are compatible with 16-bit fixed-point filtering.
  *    
  * Samples from the ADC and to the I2S interface are treated as 16 bit fixed
  * point.
  * 
  * Inspiration taken from Dynamic Calculation of FIR Filters based
  * on Bob Mailing's routines
  *****************************************************************************
  */
#include "filter.h"

//---------------------------------------------------------------
// Function to find the value of N that will satisfy:
// (a) Type II FIR filter (N odd)
// (b) Stop-band attenuation A dB
// (c) Transition bandwidth t normalised to fs
// The estimation of N is designed in accordance with
// Oppenheim & Schafer, "Discrete-Time Signal Proceesing",
// section 7.4.3.
uint kaiserFindN(float_t AdB, float_t t)
{
    uint N = (int)(fabs((AdB-8)/(4.570*M_PI*t)));
/*
    // Ensure that N is odd for a type-II FIR filter
    if (N % 2 == 0)
        N -=1;
*/
// Ensure N is even.
    if (N % 2 > 0)
        N +=1;

    return N;
}

//---------------------------------------------------------------
// Generate a lowpass filter using the Kaiser window method
// in which the number of coefficients N, stop-band attenuation A
// and stop-band frequency fs are specified.
//
// The Kaiser window is designed in accordance with
// Oppenheim & Schafer, "Discrete-Time Signal Proceesing",
// section 7.4.3.
//---------------------------------------------------------------
void wsfirKLP(float_t h[],   // h[] will be written with the filter coefficients
             const uint N, // size of the filter (number of taps)
             float_t fc,    // passband frequency
             float_t AdB)   // Stop-band attenuation in dB

{
    float_t w[N];    // window function
    float_t sinc[N]; // sinc function

    // 1. Generate Sinc function
    genSinc(sinc, N, fc);
        
    // 2. Generate the Kaiser window
    float_t alpha = (float_t)N/2;
    float_t beta = 0.0;
    
    if (AdB > 50)
        beta = 0.1102*(AdB-8.7);
    else if ( (AdB > 21) && (AdB <= 50))
        beta = 0.5842*pow((AdB-21),0.4) + 0.07886*(AdB-21);
    wKaiser(w, N, alpha, beta);

    // 3. Make lowpass filter
    for (int n = 0; n < N; n++)
    {
        h[n] = sinc[n] * w[n];
    }

    return;
}

//---------------------------------------------------------------
// Generate Kaiser highpass filter
//
// This is done by generating a lowpass filter and then
// spectrally inverting it.
//---------------------------------------------------------------
void wsfirKHP(float_t h[],    // h[] will be written with the filter coefficients
             const uint N,   // size of the filter
             float_t fc,      // cutoff frequency
             float_t AdB)     // Stopband attenuation
{
    // 1. Generate lowpass filter
    wsfirKLP(h, N, fc, AdB);

    // 2. Spectrally invert (negate all samples and add 1 to center sample) lowpass filter
    // = delta[n-((N-1)/2)] - h[n], in the time domain
    for (uint i = 0; i < N; i++)
        h[i] *= -1.0; // = 0 - h[i]

    h[(N - 1) / 2] += 1.0; // = 1 - h[(N-1)/2]

    return;
}

//---------------------------------------------------------------
// Generate Kaiser bandstop filter
//
// This is done by generating a lowpass and highpass filter
// and adding them.
//---------------------------------------------------------------
void wsfirKBS(float_t h[],    // h[] will be written with the filter taps
             const uint N,   // size of the filter             
             float_t fc1,     // low cutoff frequency
             float_t fc2,     // high cutoff frequency
             float_t AdB)     // Stop-band attenuation
{
    float_t h1[N];
    float_t h2[N];

    // 1. Generate lowpass filter at first (low) cutoff frequency
    wsfirKLP(h1, N, fc1, AdB);

    // 2. Generate highpass filter at second (high) cutoff frequency
    wsfirKHP(h2, N, fc2, AdB);

    // 3. Add the 2 filters together
    for (uint i = 0; i < N; i++)
        h[i] = h1[i] + h2[i];
  
    return;
}

//---------------------------------------------------------------
// Generate Kaiser bandpass filter
//
// This is done by generating a bandstop filter and spectrally
// inverting it.
//---------------------------------------------------------------
void wsfirKBP(float_t h[],      // h[] will be written with the filter taps
             const uint N,     // size of the filter
             float_t fc1,       // low cutoff frequency
             float_t fc2,       // high cutoff frequency
             float_t AdB)       // Stop-band attenuation
{
    // 1. Generate bandstop filter
    wsfirKBS(h, N, fc1, fc2, AdB);

    // 2. Spectrally invert bandstop (negate all, and add 1 to center sample)
    for (uint i = 0; i < N; i++)
        h[i] *= -1.0; // = 0 - h[i]
    h[(N - 1) / 2] += 1.0; // = 1 - h[(N-1)/2]

    return;
}

//---------------------------------------------------------------
// Generate sinc function - used for making lowpass filter.
//---------------------------------------------------------------
void genSinc(float_t sinc[],   // sinc[] will be written with the sinc function
    
             const uint N,          // size (number of taps)
             float_t fc)      // cutoff frequency
{
    const float_t M = N - 1;
    float_t n;

    // Generate sinc delayed by (N-1)/2
    for (uint i = 0; i < N; i++)
    {
        if (i == M / 2.0)
            sinc[i] = 2.0 * fc;    
        else
        {
            n = (float_t)i - M / 2.0;
            sinc[i] = sin(2.0 * M_PI * fc * n) / (M_PI * n);
        }
    }

    return;
}

//---------------------------------------------------------------
// Modified Bessel function of order 0, from Numerical Recipes
// in C, section 6.6 p. 237.
// This function works by fitting a polynomial to the I0(x)
// curve.
//---------------------------------------------------------------
float_t bessi0(float_t x)
// Returns the modified Bessel function I0(x) for any real x.
{
    float_t ax,ans;
    double y; // Accumulate polynomials in double precision.
    if ((ax=fabs(x)) < 3.75)
    { // Polynomial fit.
        y=x/3.75;
        y*=y;
        ans=1.0+y*(3.5156229+y*(3.0899424+y*(1.2067492
               +y*(0.2659732+y*(0.360768e-1+y*0.45813e-2)))));
    }
    else
    {
        y=3.75/ax;
        ans=(exp(ax)/sqrt(ax))*(0.39894228+y*(0.1328592e-1
        +y*(0.225319e-2+y*(-0.157565e-2+y*(0.916281e-2
        +y*(-0.2057706e-1+y*(0.2635537e-1+y*(-0.1647633e-1
        +y*0.392377e-2))))))));
    }
    return ans;
}

//---------------------------------------------------------------
// Generate window function (Kaiser)
// The Kaiser window has arguments alpha and beta that determine
// the shape of the window. The modified Bessel function of the
// first kind, 0'th order I0() is used. It is defined above bessi0().
//---------------------------------------------------------------
void wKaiser(float_t w[],
             const uint N,
             float_t alpha,
             float_t beta
            )
{
    for(uint i=0; i < N; i++)
    {
        w[i] = bessi0( beta * pow(1 - pow((i-alpha)/alpha,2),0.5) ) / bessi0(beta);
    }
}