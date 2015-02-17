/*
 * srf08.c
 *
 *  Created on: 30.11.2010
 *      Author: philipp
 */

#include <avr/io.h>
#include "twimaster.h"
#include "srf08.h"
#include "uart.h"
#include "util.h"

#define SLAVE_ADRESS 0xEA

void initSRF(void)
{
	i2c_start(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x02);
	i2c_write(70);

	i2c_rep_start(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x01);
	i2c_write(0x05);

	i2c_stop();
}

unsigned char srf08_ready(void)
{
	if(!(i2c_start(SLAVE_ADRESS + I2C_WRITE)))
	{
		i2c_stop();
		return 1;
	}

	i2c_stop();
	return 0;
}


void setAmpValue(void)
{
	while(!uart1_data_received());

	unsigned char c = (unsigned char) uart1_getc();

	i2c_start(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x01);
	i2c_write(c);
	i2c_stop();
}

void setSpecAmpValue(unsigned char value)
{
	i2c_start(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x01);
	i2c_write(value);
	i2c_stop();
}

void setRangeValue(void)
{
	while(!uart1_data_received());

	unsigned char c = (unsigned char) uart1_getc();

	i2c_start(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x02);
	i2c_write(c);
	i2c_stop();
}

void setSpecRangeValue(unsigned char value)
{
	i2c_start(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x02);
	i2c_write(value);
	i2c_stop();
}

int getSensorRevision(void)
{
	unsigned char value;
    i2c_start(SLAVE_ADRESS + I2C_WRITE);
    i2c_write(0x00);
    i2c_rep_start(SLAVE_ADRESS + I2C_READ);
    value = i2c_readNak();
    i2c_stop();

    return value;
}

unsigned int getRange(void)
{
	unsigned int result = 0;

	i2c_rep_start(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x00);
	i2c_write(START_MEASUREMENT_CM);
	i2c_stop();

	while(!srf08_ready());

	i2c_start_wait(SLAVE_ADRESS+I2C_WRITE);
	i2c_write(0x02);

	i2c_rep_start(SLAVE_ADRESS+I2C_READ);
	result = ((i2c_readAck())<<8);
	result += (i2c_readNak());

	return result;
}

void autoCalibration(void)
{
	int i;
	int j;

	setRangeValue();

	while(!uart1_data_received());

	unsigned char c = (unsigned char) uart1_getc();

	for(i = 0; i < 32; i++)
	{
		setSpecAmpValue(i);

		for(j = 0; j < c; j++)
		{
			unsigned int result = getRange();
			printValue(result);
		}
	}
}
