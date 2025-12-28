/*
 * I2C_functions.c
 *
 *  Created on: Dec 28, 2025
 *      Author: Matteo
 */

#include "main.h"

extern float acceleration[3];

void convert_accelerations(uint8_t* i2c_buffer_pointer)
{
	int16_t temp;

	// raw X acceleration
	temp = ((uint16_t)i2c_buffer_pointer[1]<<8) | ((uint16_t)i2c_buffer_pointer[0]);
	acceleration[0] = ((float)temp) * 0.061f; // convert to mg (+/-2g)

	// raw Y acceleration
	temp = ((uint16_t)i2c_buffer_pointer[3]<<8) | ((uint16_t)i2c_buffer_pointer[2]);
	acceleration[1] = ((float)temp) * 0.061f; // convert to mg

	// raw Z acceleration
	temp = ((uint16_t)i2c_buffer_pointer[5]<<8) | ((uint16_t)i2c_buffer_pointer[4]);
	acceleration[2] = ((float)temp) * 0.061f; // convert to mg
}

