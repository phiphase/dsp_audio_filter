/*
  ******************************************************************************
  * Copyright (C) 2025 Adrian P. Nash, G4ZHZ, github/phiphase
  * File:    ads1015.c
  * Description:
  * Implements a simple driver for the ADS1015 ADC using the I2C interface
  *****************************************************************************
  */
 #include <stdio.h>
 #include "ads1015.h"
 #include "hardware/i2c.h"

bool ads1015_init()
{
    int r;  // return code.
    uint8_t rxData;

    gpio_set_function(I2C1_SDA_GPIO, GPIO_FUNC_I2C);
    gpio_set_function(I2C1_SCK_GPIO, GPIO_FUNC_I2C);
	gpio_pull_up(I2C1_SDA_GPIO);
	gpio_pull_up(I2C1_SCK_GPIO);

    // Initialise the I2C hardware in the pico
    i2c_init(I2C_DEV, I2C_CLK_SPEED * 1000);

    r = i2c_read_timeout_us(I2C_DEV, ADS1015_ADDR_GND, &rxData, 1, false, I2C_TIMEOUT);
    if(r < 0)
    {
        printf("Failed to initialise ADS1015 I2C\n");
        return false;
    }
    return true;
}

int16_t read_adc(uint8_t ch)
{
    uint8_t buffer[3];

    /* ADS1015 CONFIGURATION
    +----------------------------------------------------- OS bit 15 = 1. Start single conversion
    |     +----------------------------------------------- MUX[2:0] = rrr set by ADC channel. ADC0: 100: ADC1: 101, ADC2: 110, ADC3: 111
    |     |        +-------------------------------------- PGA[2:0] = 000. PGA = +/- 6.144 V FSR
    |     |        |      +------------------------------- MODE = 1. Single-shot mode
    |     |        |      |     +------------------------- DR[2:0] = 100. 1600sps data rate
    |     |        |      |     |     +------------------- COMP_MODE = 0. Traditional comparator mode
    |     |        |      |     |     |    +-------------- COMP_POL = 0 Alrt/Rdy is active low
    |     |        |      |     |     |    |   +---------- COMP_LAT = 0. Latching comparator = non-latched
    |     |        |      |     |     |    |   |    +----- COMP_QUE[1:0] = 11. Disable comparator and alrt/rdy flag
    |     |        |      |     |     |    |   |    |
    -   -----    -----    -   -----   -    -   -   --- 
    1   r r r  \ 0 0 0    1 \ 1 0 0   0  \ 0   0   1 1
  
    0x 8 | rrr\1\8\3
    */
    uint16_t config = 0x8183;

    switch(ch)
    {
        case 0:
            config |= ADC0_MUX;        
            break;
        case 1:
            config |= ADC1_MUX;            
            break;
        case 2:
            config |= ADC2_MUX;            
            break;
        case 3:
            config |= ADC3_MUX;            
            break;
        default:
            printf("Invalid ADC channel\n");
            return 0;
    }

    // Start the ADC conversion
    buffer[0] = (uint8_t)ADC_REG_CONFIG;  // Register address
	buffer[1] = (uint8_t)(config >> 8);   // MSB
	buffer[2] = (uint8_t)(config & 0xFF); // LSB
    i2c_write_timeout_us(I2C_DEV, ADS1015_ADDR_GND, buffer, 3, false, I2C_TIMEOUT);

    // Wait for the conversion to complete.
    buffer[0] = ADC_REG_CONFIG;
    i2c_write_timeout_us(I2C_DEV, ADS1015_ADDR_GND, buffer, 1, false, I2C_TIMEOUT);
    do
        i2c_read_timeout_us(I2C_DEV, ADS1015_ADDR_GND, buffer, 1, false, I2C_TIMEOUT);
    while((buffer[0] & 0x80) != 0);

    // Get the result of the A/D conversion
    buffer[0] = ADC_REG_CONV_RESULT;
    i2c_write_timeout_us(I2C_DEV, ADS1015_ADDR_GND, buffer, 1, false, I2C_TIMEOUT);    
    i2c_read_timeout_us(I2C_DEV, ADS1015_ADDR_GND, buffer, 2, false, I2C_TIMEOUT);
    uint16_t result = (buffer[0] << 8 | buffer[1] & 0xFF) >> 4;

	// Convert to signed (2's complement)
	if (result > 0x07FF)
	{
		// negative number - extend the sign to 16th bit
		result |= 0xF000;
	}
	return (int16_t)result;
}
