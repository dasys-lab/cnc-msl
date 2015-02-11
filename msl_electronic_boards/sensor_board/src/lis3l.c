#include "lis3l.h"
#include "twimaster.h"

#define SLAVE_ADRESS 0x3A

void initLis3l(void)
{
	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x20);
	i2c_write(135);
	i2c_stop();

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x21);
	i2c_write(64);
	i2c_stop();
}

int whoAmI(void)
{
	unsigned char value;

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x0F);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);
	value = i2c_readNak();
	i2c_stop();

	return value;
}

int readX(void)
{
	int value;

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x29);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);
	value = (i2c_readNak() << 8);
	i2c_stop();

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x28);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);
	value += i2c_readNak();
	i2c_stop();

	return value;
}

int readY(void)
{
	int value;

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x2B);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);
	value = (i2c_readNak() << 8);
	i2c_stop();

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x2A);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);
	value += i2c_readNak();
	i2c_stop();

	return value;
}

int readZ(void)
{
	int value;

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x2D);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);
	value = (i2c_readNak() << 8);
	i2c_stop();

	i2c_start(SLAVE_ADRESS + I2C_WRITE);
	i2c_write(0x2C);

	i2c_rep_start(SLAVE_ADRESS + I2C_READ);
	value += i2c_readNak();
	i2c_stop();

	return value;
}
