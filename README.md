# G4ZHZ DSP AUDIO FILTER

This DSP audio filter project has been developed to provide:
- Variable bandwidth audio filtering (e.g. for CW or interference reduction)
- Automatic Line Enhancement (ALE) for noise reduction or automatich notch filtering
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
Audio filter can either operate as fL (low frequency) or fH (high frequency) or in FC (centre frequency) and bandwidth (BW).
- Frequency range (FC): 0-2000 Hz
- Bandwidth range (BW): 0-2000 Hz

Automatic Line Enhancer (ALE) in/out
ALE operates either in noise reduction (NR) mode or automatic notch cancelling (AN) mode.

800 Hz CW side tone. Volume internally adjustable

CW Keyer: 5-55WPM

Power, 3-12 V. PP3 9V battery or external power via 2.5mm DC jack

Current (headphones) ~24 mA

## FRONT PANEL
Front Panel Controls and indicators:
- Power On/off
- Filter In/Out
- ALE In/Out
- Tune (CW output on)
- Audio filter centre frequency (FC) analogue control 0-2000 Hz
- Audio filter badwidth (BW) analogue control 0-2000 Hz
- CW Keyer speed 5-55 WPM.
- Power ON LED (Green)
- CW Keyed LED (Red)


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

## PHOTOS
Below are some photos of the finished DSP audio filter.
![20251014_221011](https://github.com/user-attachments/assets/fc971698-db09-4ba0-b9a3-33f07d74c2da)
![20251014_221031](https://github.com/user-attachments/assets/8a0bb6a7-eba8-4af0-a554-8d81f498e0d7)
![20251014_221043](https://github.com/user-attachments/assets/32d21914-bdb2-434a-b174-c0aac0436e3e)
![20251015_191848](https://github.com/user-attachments/assets/58642834-510d-435b-9d93-0bf74ced642b)
