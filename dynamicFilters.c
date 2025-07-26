/*
  ******************************************************************************
  * File    dynamicFilters.c
  * Author: Adrian Nash, G4ZHZ
  *         from Giuseppe Callipo's Github project SimpleAudioFilter09
  *         ik8yfw@libero.it
  * Description:
  *    Dynamic Calculation of FIR Filters based on Bob Mailing's routines
  *    NOTE: This file is part of RadioDSP MINI project.
  * Converted double to float so that the Pico 2's FPU can be used.
  * Added Kaiser window versions of the filter that allow the stop-band
  * attenuation and transition bandwidth to be specified. Added a function,
  * kaiserFindN(float AdB, float t) that returns the number of taps N required.
  * 
  ******************************************************************************
  */
#include "dynamicFilters.h"

float  fir_tmp[NUM_COEFFICIENTS];   
                       

// Temp array used for coefficient calculations which are performed in 64bit


//---------------------------------------------------------------
// Generate lowpass filter
//
// This is done by generating a sinc function and then windowing it
void wsfirLP(float h[],    // h[] will be written with the filter coefficients
             const int N,    // size of the filter (number of taps)
             const int WINDOW, // window function (W_BLACKMAN, W_HANNING, etc.)
             float fc)    // corner frequency
{
    int i;
    float w[N];    // window function
    float sinc[N]; // sinc function

    // 1. Generate Sinc function
    genSinc(sinc, N, fc);

    // 2. Generate Window function
    if (WINDOW == W_BLACKMAN)
        wBlackman(w, N);
    else if (WINDOW == W_HANNING)
        wHanning(w, N);
    else if (WINDOW == W_HAMMING)
        wHamming(w, N);

    // 3. Make lowpass filter
    for (i = 0; i < N; i++)
        h[i] = sinc[i] * w[i];

    return;
}


//---------------------------------------------------------------
// Function to find the value of N that will satisfy:
// (a) Type II FIR filter (N odd)
// (b) Stop-band attenuation A dB
// (c) Transition bandwidth t normalised to fs
// The estimation of N is designed in accordance with
// Oppenheim & Schafer, "Discrete-Time Signal Proceesing",
// section 7.4.3.
int kaiserFindN(float AdB, float t)
{
    int N = (int)(fabs((AdB-8)/(4.570*M_PI*t)));
    if (N % 2 == 0)
        N -=1;
    return N;
}

//---------------------------------------------------------------
// Generate a lowpass filter using the Kaiser window method
// in which the number of coefficients N, stop-band attenuation A
// and stop-band frequency fs are specified.
//
// The Kaiser window is designed in accordance with Oppenheim & Schafer,
// "Discrete-Time Signal Proceesing", section 7.4.3.
// Note: This filter returns an error code. If 0, no error. If
// negative an error occurred.
// 
void wsfirKLP(float h[],   // h[] will be written with the filter coefficients
             const int N, // size of the filter (number of taps)
             float fc,    // passband frequency
             float AdB)   // Stop-band attenuation in dB

{
    int i;
    float w[N];    // window function
    float sinc[N]; // sinc function

    // 1. Generate Sinc function
    genSinc(sinc, N, fc);
        
    // 2. Generate the Kaiser window
    float alpha = (float)N/2;
    float beta = 0.0;
    
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
// This is done by generating a lowpass filter and then spectrally inverting it
void wsfirKHP(float h[],    // h[] will be written with the filter coefficients
             const int N,   // size of the filter
             float fc,      // cutoff frequency
             float AdB)     // Stopband attenuation
{
    int i;

    // 1. Generate lowpass filter
    wsfirKLP(h, N, fc, AdB);

    // 2. Spectrally invert (negate all samples and add 1 to center sample) lowpass filter
    // = delta[n-((N-1)/2)] - h[n], in the time domain
    for (i = 0; i < N; i++)
        h[i] *= -1.0; // = 0 - h[i]

    h[(N - 1) / 2] += 1.0; // = 1 - h[(N-1)/2]

    return;
}

//---------------------------------------------------------------
// Generate Kaiser bandstop filter
//
// This is done by generating a lowpass and highpass filter and adding them
void wsfirKBS(float h[],    // h[] will be written with the filter taps
             const int N,   // size of the filter             
             float fc1,     // low cutoff frequency
             float fc2,     // high cutoff frequency
             float AdB)     // Stop-band attenuation
{
    int i;
    float h1 [N];
    float h2 [N];

    // 1. Generate lowpass filter at first (low) cutoff frequency
    wsfirKLP(h1, N, fc1, AdB);

    // 2. Generate highpass filter at second (high) cutoff frequency
    wsfirKHP(h2, N, fc2, AdB);

    // 3. Add the 2 filters together
    for (i = 0; i < N; i++)
        h[i] = h1[i] + h2[i];
  
    return;
}

//---------------------------------------------------------------
// Generate Kaiser bandpass filter
//
// This is done by generating a bandstop filter and spectrally inverting it
void wsfirKBP(float h[],      // h[] will be written with the filter taps
             const int N,     // size of the filter
             float fc1,       // low cutoff frequency
             float fc2,       // high cutoff frequency
             float AdB)       // Stop-band attenuation
{
    int i;

    // 1. Generate bandstop filter
    wsfirKBS(h, N, fc1, fc2, AdB);

    // 2. Spectrally invert bandstop (negate all, and add 1 to center sample)
    for (i = 0; i < N; i++)
        h[i] *= -1.0; // = 0 - h[i]
    h[(N - 1) / 2] += 1.0; // = 1 - h[(N-1)/2]

    return;
}

//---------------------------------------------------------------
// Generate highpass filter
//
// This is done by generating a lowpass filter and then spectrally inverting it
void wsfirHP(float h[],    // h[] will be written with the filter coefficients
             const int N,    // size of the filter
             const int WINDOW, // window function (W_BLACKMAN, W_HANNING, etc.)
             float fc)  // cutoff frequency
{
    int i;

    // 1. Generate lowpass filter
    wsfirLP(h, N, WINDOW, fc);

    // 2. Spectrally invert (negate all samples and add 1 to center sample) lowpass filter
    // = delta[n-((N-1)/2)] - h[n], in the time domain
    for (i = 0; i < N; i++)
        h[i] *= -1.0; // = 0 - h[i]

    h[(N - 1) / 2] += 1.0; // = 1 - h[(N-1)/2]

    return;
}


//---------------------------------------------------------------
// Generate bandstop filter
//
// This is done by generating a lowpass and highpass filter and adding them
void wsfirBS(float h[],    // h[] will be written with the filter taps
             const int N,    // size of the filter
             const int WINDOW, // window function (W_BLACKMAN, W_HANNING, etc.)
             float fc1, // low cutoff frequency
             float fc2) // high cutoff frequency
{
    int i;
    float h1 [N];
    float h2 [N];

    // 1. Generate lowpass filter at first (low) cutoff frequency
    wsfirLP(h1, N, WINDOW, fc1);

    // 2. Generate highpass filter at second (high) cutoff frequency
    wsfirHP(h2, N, WINDOW, fc2);

    // 3. Add the 2 filters together
    for (i = 0; i < N; i++)
        h[i] = h1[i] + h2[i];
  
    return;
}

//---------------------------------------------------------------
// Generate bandpass filter
//
// This is done by generating a bandstop filter and spectrally inverting it
void wsfirBP(float h[],        // h[] will be written with the filter taps
             const int N,            // size of the filter
             const int WINDOW,       // window function (W_BLACKMAN, W_HANNING, etc.)
             float fc1,       // low cutoff frequency
             float fc2)       // high cutoff frequency
{
    int i;

    // 1. Generate bandstop filter
    wsfirBS(h, N, WINDOW, fc1, fc2);

    // 2. Spectrally invert bandstop (negate all, and add 1 to center sample)
    for (i = 0; i < N; i++)
        h[i] *= -1.0; // = 0 - h[i]
    h[(N - 1) / 2] += 1.0; // = 1 - h[(N-1)/2]

    return;
}

//---------------------------------------------------------------
// Generate sinc function - used for making lowpass filter
void genSinc(float sinc[],   // sinc[] will be written with the sinc function
             const int N,          // size (number of taps)
             float fc)      // cutoff frequency
{
    int i;
    const float M = N - 1;
    float n;

    // Generate sinc delayed by (N-1)/2
    for (i = 0; i < N; i++)
    {
        if (i == M / 2.0)
            sinc[i] = 2.0 * fc;    
        else
        {
            n = (float)i - M / 2.0;
            sinc[i] = sin(2.0 * M_PI * fc * n) / (M_PI * n);
        }
    }

    return;
}


//---------------------------------------------------------------
// Generate window function (Blackman)
void wBlackman(float w[],    // w[] will be written with the Blackman window
               const int N)        // size of the window
{
    int i;
    const float M = N - 1;

    for (i = 0; i < N; i++)
        w[i] = 0.42 - (0.5 * cos(2.0 * M_PI * (float)i / M)) + (0.08 * cos(4.0 * M_PI * (float)i / M));

    return;
}

//---------------------------------------------------------------
// Generate window function (Hann)
void wHanning(float w[],   // w[] will be written with the Hann window
              const int N)       // size of the window
{
    int i;
    const float M = N - 1;

    for (i = 0; i < N; i++)
        w[i] = 0.5 * (1.0 - cos(2.0 * M_PI * (float)i / M));

    return;
}


//---------------------------------------------------------------
// Generate window function (Hamming)
void wHamming(float w[],   // w[] will be written with the Hamming window
              const int N)       // size of the window
{
    int i;
    const float M = N - 1;

    for (i = 0; i < N; i++)
        w[i] = 0.54 - (0.46 * cos(2.0 * M_PI * (float)i / M));

    return;
}

//---------------------------------------------------------------
// Modified Bessel function of order 0, from Numerical Recipes
// in C, section 6.6 p. 237.
// This function works by fitting a polynomial to the I0(x)
// curve.
float bessi0(float x)
// Returns the modified Bessel function I0(x) for any real x.
{
    float ax,ans;
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
void wKaiser(float w[],
             const int N,
             float alpha,
             float beta
            )
{
    for(int n=0; n < N;n++)
    {
        w[n] = bessi0( beta * pow(1 - pow((n-alpha)/alpha,2),0.5) ) / bessi0(beta);
    }
}

//---------------------------------------------------------------
void coeffConvert(float in[], float out[], const int N)
{
    for (int j = 0; j < N; j++)
        out[j] = in[j];
}

void convertCoeffToInt16(float in[], int out[], const int N)
{
    int acc =0;
    for (int j = 0; j < N; j++)
    {
        acc = (int)(in[j] * 32768.0);
        out[j] = acc;  // >> 16;   
    }
}

//---------------------------------------------------------------
void lowpass(float h[], const int N, const int WINDOW, float fc)
{
    wsfirLP(fir_tmp, N, WINDOW, fc );
    coeffConvert(fir_tmp, h, N);
}

//---------------------------------------------------------------
void highpass(float h[], const int N, const int WINDOW, float fc)
{
    wsfirHP(fir_tmp, N, WINDOW, fc);
    coeffConvert(fir_tmp, h, N);
}

//---------------------------------------------------------------
void bandpass(float h[], const int N, const int WINDOW, float fc1, float fc2)
{
    wsfirBP(fir_tmp, N, WINDOW, fc1, fc2 );
    coeffConvert(fir_tmp, h, N);
}

//---------------------------------------------------------------
void bandstop(float h[], const int N, const int WINDOW, float fc1, float fc2)
{
    wsfirBS(fir_tmp, N, WINDOW, fc1, fc2 );
    coeffConvert(fir_tmp, h, N);
}


//---------------------------------------------------------------
void audioFilter(float h[], const int N, const int TYPE, const int WINDOW, float fc1, float fc2, float  fs )
{
    switch (TYPE)
    {
        case ID_LOWPASS:
            lowpass(h, N, WINDOW, fc1/fs);
            break;
        case ID_HIGHPASS:
            highpass(h, N, WINDOW, fc1/fs);
            break;
        case ID_BANDPASS:
            bandpass(h, N, WINDOW, fc1/fs, fc2/fs);
            break;
        case ID_BANDSTOP:
            bandstop(h, N, WINDOW, fc1/fs, fc2/fs);
            break;
        default:
            break;
    }
}