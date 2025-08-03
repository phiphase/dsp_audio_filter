/*
  ******************************************************************************
  * Copyright (C) 2025 Adrian P. Nash, G4ZHZ, github/phiphase
  * File:    ads1015.h
  * Description:
  * Include file for ads1015.h
  *****************************************************************************
  */
#ifndef __ADS1015_H__
#define __ADS1015_H__
#include "pico/stdlib.h"  // Included so that the int16_t/uint16_t/float32_t32_t types are available.

#define I2C_DEV i2c1
#define I2C1_SDA_GPIO 14       // GPIO pin for the I2C data (physical pin 19 on rpi pico2 module)
#define I2C1_SCK_GPIO 15       // GPIO pin for the I2C clock (physical pin 20 on rpi pico2 module)
#define I2C_CLK_SPEED 100      // I2C clock speed, kHz
#define ADS1015_ADDR_GND 0x48  // 7-bit address for the ADS1015 (lsb is the rd/wr bit)
#define I2C_TIMEOUT 50000      // Timeout, us
#define ADC0_MUX 0x4000        // Mux configuration setting for ADC0
#define ADC1_MUX 0x5000        // Mux configuration setting for ADC1
#define ADC2_MUX 0x6000        // Mux configuration setting for ADC2
#define ADC3_MUX 0x7000        // Mux configuration setting for ADC3

// ADS1015 registers that we use
#define ADC_REG_CONV_RESULT 0x00
#define ADC_REG_CONFIG      0x01
#define ADC_REG_LOW_THRESH  0x02
#define ADC_REG_HIGH_THRESH 0x03

// Initialise the ADS1015.
// Speed = 100 kHz
// I2C1 interface used (GPIO24 for SDA, GPIO25 for SCL)
// Master mode
// Timeout = 50,000 us
// ADDRESS_GND
//
// Returns false on failure, else true
bool ads1015_init();

// Read the ADC adc_ch = 0, 1, 2 or 3. Returns the ADC value.
int16_t read_adc(uint8_t adc_ch);

 #endif