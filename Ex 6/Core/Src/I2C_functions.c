/*
 * I2C_functions.c
 *
 *  Created on: Dec 28, 2025
 *      Author: Matteo
 */

#include "main.h"

extern float acceleration[3];
extern float temperature;
extern float angular_vel[3];

void convert_accelerations(uint8_t* i2c_buffer_pointer)
{
	int16_t temp;

	// temperature
	temp = ((uint16_t)i2c_buffer_pointer[1]<<8) | ((uint16_t)i2c_buffer_pointer[0]);
	temperature = (((float)temp)/256.0f) + 25.0f; // convert to degrees celsius


	// velocity
	// raw X velocity
	temp = ((uint16_t)i2c_buffer_pointer[3]<<8) | ((uint16_t)i2c_buffer_pointer[2]);
	angular_vel[0] = ((float)temp) * 8.75f; // convert to milli-degrees per second

	// raw Y velocity
	temp = ((uint16_t)i2c_buffer_pointer[5]<<8) | ((uint16_t)i2c_buffer_pointer[4]);
	angular_vel[1] = ((float)temp) * 8.75f; // convert to milli-degrees per second

	// raw Z velocity
	temp = ((uint16_t)i2c_buffer_pointer[7]<<8) | ((uint16_t)i2c_buffer_pointer[6]);
	angular_vel[2] = ((float)temp) * 8.75f; // convert to milli-degrees per second


	// acceleration
	// raw X acceleration
	temp = ((uint16_t)i2c_buffer_pointer[9]<<8) | ((uint16_t)i2c_buffer_pointer[8]);
	acceleration[0] = ((float)temp) * 0.061f; // convert to mg (+/-2g)

	// raw Y acceleration
	temp = ((uint16_t)i2c_buffer_pointer[11]<<8) | ((uint16_t)i2c_buffer_pointer[10]);
	acceleration[1] = ((float)temp) * 0.061f; // convert to mg

	// raw Z acceleration
	temp = ((uint16_t)i2c_buffer_pointer[13]<<8) | ((uint16_t)i2c_buffer_pointer[12]);
	acceleration[2] = ((float)temp) * 0.061f; // convert to mg
}

