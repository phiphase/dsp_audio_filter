# G4ZHZ DSP AUDIO FILTER

This DSP audio filter project has been developed to provide:
- Variable bandwidth audio filtering (e.g. for CW or interference reduction)
- Automatic Line Enhancement (ALE) for noise reduction or tone elimination
- CW sidetone generation
- Electronic paddle keyer for CW.

It is intended to be used between the headphones/speaker and the headphone output of an HF radio that has an IF bandwidth that
is too wide for CW use. The audio filter is based on the Rasberry PI Pico2 module and uses the ARM CMSIS DSP library.
It comprises Rasberry Pi or Adafruit modules which are readily available:
- Rasberry PI Pico-2
- Adafruit I2S DAC module
- Adafruit Verta boost/buck DC/DC converter module
- Adafruit ADS1015 4-channel ADC Module
- Adafruit Class D speaker amplifier

There are also a handful of other components (resistors, capacitors, 2N2222 transistors, relay etc.)
The PICO-2's 12-bit internal ADC is used for audio input. DMA channels, double buffering, PIO I2S interface and the dual-processor
message-posting FIFO are used, so the DSP CPU is only loaded to around 4% leaving loads of processing capability for other DSP
tasks like FFT/IFFT or advanced noise reduction in the future. One processor core does the DSP. The other does the user interface.

## SPECIFICATIONS
The specifications are as follows:

Audio filter in/out
Audio filter can either operate as fL (low frequency) or fH (high frequency) or in fc (centre frequency) and bandwidth (BW).
- Frequency range: 300-3300 Hz
- Bandwidth range: 50 Hz to 3000 Hz

Automatic Line Enhancer (ALE) in/out
ALE operates either in noise reduction mode or automatic tone cancelling mode.

800 Hz CW side tone. Volume internally adjustable

CW Keyer: 5-50WPM

Power, 3-12 V. PP3 9V battery or external power via 2.5mm DC jack

Current (headphones) ~24 mA

## FRONT PANEL
Front Panel Controls and indicators:
- Power On/off
- Filter In/Out
- ALE In/Out
- Tune (CW output on)
- Audio filter fL/fc analogue control 300-3300 Hz
- Audio filter fH/BW analogue control 300-3300 Hz
- CW Keyer speed 5-50 WPM.
- Power ON LED (Green)
- CW Keyed LED (Red)
- Filter in LED (Yellow)
- ALE in LED (Yellow)

## REAR PANEL
Read Panel Connectors
- 2.5mm DC Jack centre +, 3V - 12V, 100mA maximum
- 3.5mm Audio in (~600 Ohm)
- 3.5mm Headphone out(32 Ohm or higher)
- 3.5mm Key in
- 3.5mm Paddle in: Dot on tip, Dash on ring
- 3.5mm Key out
- RCA Phono External speaker out (8 Ohm)

## BLOCK DIAGRAM
Below is the block diagram of the DSP audio filter.

<img width="4512" height="3064" alt="image" src="https://github.com/user-attachments/assets/ea97d940-51c5-4123-85bd-feb25c82b5c4" />



