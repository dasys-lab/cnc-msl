#include "cap.h"
#include "twimaster.h"
#include "uart.h"
#include "util.h"
#include <stdio.h>
#include <util/delay.h>

#define SLAVE_ADRESS 0x90

char buf[BUFFERLENGTH];

void initCap(void)
{
	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0xBF);
	i2c_stop();

	_delay_ms(1);

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x09);
	i2c_write(0x08);
	i2c_stop();

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x07);
	i2c_write(0x80);
	i2c_stop();

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x0A);
	i2c_write(0x01 | (7 << 3));
	i2c_stop();

	_delay_ms(150);

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x0B);
	i2c_write(0x00 | 39);
	i2c_stop();
}

long read(void)
{
	long value;
	double realValue;

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x01);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);

	value = ((((long) i2c_readAck()) << 16) + (((long) i2c_readAck()) << 8) + (long) i2c_readNak());
	i2c_stop();

	return value;
}


