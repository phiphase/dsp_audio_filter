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

## CONSTRUCTION
This MK-1 version is intended to be built on Verboard. It is pretty straightforward to build because Adafruit modules are used.
So, having built the prototype, the following advice is given.
* Use headers and leads (available from the Pihut)
* Make sure the Verboard is large enough to hold all the modules (the board I used was a little small).
* Panel mount all the controls.
* Do fit the RF filters on the rear panel connections
* Make sure you use a conductive (Aluminium) box. The box I used appeared to have some sort of insulating coating on it so I had to use a needle file to remove the coating to get an electrical contact!
* The speaker connection (phono plug on the prototype) + and - must be isolated from ground. This is not easy to do.
* The battery is held in place with a Velcro coin.
* The RF chokes may be very easily damaged by heat. I used 180 mA rated chokes and most of these went open circuit upon soldering and had to be replaced.
* For the front and rear panel legends I used transparent Gekko paper that enables the legends to be printed using an ink-jet printer. However, cut each legend out individually and carefully place on the drilled but un-fitted panels if possible. Don't do what I did and print the whole front and rear panels on Gekko paper!
* If possible, wire up and fit the panel components to the panels before fitting the panels and wiring it all up i.e. use separate panels.
* Make sure the Pico-2 module's USB and debug connectors can be easily accessed.

## PROGRAMMING
It is probably best to follow the Pico-2 getting started guide https://datasheets.raspberrypi.com/pico/getting-started-with-pico.pdf. Start by installing Microsoft VS Code. Instal the Pico project plug-in, clone the software repo and compile/install the code. You'll need to flash the Pico-2 using the USB cable. Alternatively, download dsp_audio_filter_build.zip, de-compress it and use your favourite tool to upload to the target.

## PHOTOS
Below are some photos of the finished DSP audio filter.
![20251014_221011](https://github.com/user-attachments/assets/fc971698-db09-4ba0-b9a3-33f07d74c2da)
![20251014_221031](https://github.com/user-attachments/assets/8a0bb6a7-eba8-4af0-a554-8d81f498e0d7)
![20251014_221043](https://github.com/user-attachments/assets/32d21914-bdb2-434a-b174-c0aac0436e3e)
![20251015_191848](https://github.com/user-attachments/assets/58642834-510d-435b-9d93-0bf74ced642b)
