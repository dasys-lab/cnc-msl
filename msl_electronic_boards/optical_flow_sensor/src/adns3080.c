#include <avr/io.h>
#include "adns3080.h"

void configuration_read_and_write_test(void)
{
	reset();
	
	uart1_puts("Configuration Read/Write Test:\r\n");
	
	
	uint8_t val = getConfigurationBits();
	
	char tmpbuff[100];
	sprintf(tmpbuff,"Read default: %u\r\n",val);
	uart1_puts(tmpbuff);
	
	//_delay_us(50);
	setConfigurationBits(0x59);
	
	char tmpbuff2[100];
	uint8_t val2 = getConfigurationBits();
	sprintf(tmpbuff2,"Read after set: %u\r\n",val2);
	uart1_puts(tmpbuff2);
}

uint8_t getConfigurationBits(void)
{
	return read(CONFIGURATION_BITS);
}

void getFrame(uint8_t *image)
{
	reset();
	
	uint8_t regValue;
	bool isFirstPixel = false;
	
	write(FRAME_CAPTURE,0x83);
	// wait 3 frame periods + 10 us for frame to be captured
	_delay_us(1510);	// min frame speed is 2000 frames/second so 1 frame = 500 us.  so 500 x 3 + 10 = 1510
	// 	uint16_t count = 0;
	for(int i=0; i<RESOLUTION;)
	{
		for(int j=0; j<RESOLUTION;)
		{
			// 			count++;
			regValue = read(FRAME_CAPTURE);
			if( !isFirstPixel && (regValue & 0x40) == 0 )
			{
				//uart1_puts("failed to find first pixel!\r\n");
				i=0;
				j=0;
				break;
			}
			else
			{
				if( (regValue & 0x40) == 0x40 )
				{
					uart1_puts("first");
				}
				//uart1_puts("found first pixel!");
				isFirstPixel = true;
				//pixelValue = ( regValue << 2);
				*image = ( regValue << 2);
				//*image = regValue;
				image++;	// next array cell address
			}
			_delay_us(50);
			j++;
		}
		if( isFirstPixel )
		{
			i++;
		}
	}
	reset();
	
	// 	uart1_puts("count :");
	// 	char tmpbuff[1];
	// 	sprintf(tmpbuff,"%u\r\n",count);
	// 	uart1_puts(tmpbuff);
}

void getFrameBurst(uint8_t *image)
{
	
	uint8_t regValue;
	
	write(FRAME_CAPTURE,0x83);
	// wait 3 frame periods + 10 us for frame to be captured
	// min frame speed is 2000 frames/second so 1 frame = 500 us.  so 500 x 3 + 10 = 1510
	_delay_us(1510);	// wait t_CAPTURE
	RESET(P_NCS);
	spi_exch(PIXEL_BURST);
	_delay_us(50);		// wait t_SRAD
	
	for(int i=0; i<900; i++)	// max. 1536 Pixels
	{
		regValue = spi_exch(0x00);
		image[i] = (regValue << 2);
		_delay_us(10);	// wait t_LOAD
	}
	
	SET(P_NCS);
	_delay_us(4);	// wait t_BEXIT
}

uint8_t getInverseProductId(void)
{
	return read(INVERSE_PRODUCT_ID);
}

void getMotion(int8_t *motion)
{
	uint8_t val;
	//  	while( 1 )
	{
		val = read(MOTION);
 		char moBuff[10];
 		sprintf(moBuff,"%u\t%u\r\n",val & 0x10, val & 0x80);
 		uart1_puts(moBuff);
		
		if( (val & 0x80) != 0 )
		{
			motion[0] = (int8_t)read(DELTA_X);
			_delay_us(50);
			motion[1] = (int8_t)read(DELTA_Y);
			//  			break;
		}
		else
		{
			motion[0] = 0;
			motion[1] = 0;
		}
	}
	// 	write(MOTION_CLEAR,0xFF);
}

void getMotionBurst(uint8_t *burst)
{
	RESET(P_NCS);
	
	spi_exch(MOTION_BURST);
	_delay_us(75);	// wait t_SRAD-MOT
	
	// read all 7 bytes (Motion, Delta_X, Delta_Y, SQUAL, Shutter_Upper, Shutter_Lower, Maximum Pixels)
	for(uint8_t i=0; i<7; i++)
	{
		burst[i] = spi_exch(0x00);
	}
	
	SET(P_NCS);
	_delay_us(1);
}

uint8_t getProductId(void)
{
	return read(PRODUCT_ID);
}

void printFrame(uint8_t *image)
{
	for(int i=0; i<RESOLUTION; i++)
	{
		for(int j=0; j<RESOLUTION; j++)
		{
			char tmpbuff[5];
			if( j == RESOLUTION -1 )
			{
				sprintf(tmpbuff,"%u\r\n",*image);
			}
			else
			{
				sprintf(tmpbuff,"%u;",*image);
			}
			uart1_puts(tmpbuff);
			image++;
		}
	}
	uart1_puts("-\r\n");
}

void printFrameBinary(uint8_t *image)
{
	for(int i=0; i<RESOLUTION; i++)
	{
		for(int j=0; j<RESOLUTION; j++)
		{
			uart1_puts(*image);
			image++;
		}
	}
	//uart1_puts(EOF_FRAME);
}

void printFrameFromPixelBuffer(uint8_t *image)
{
	uint16_t begin = 2000;
	for(uint16_t i=0; i<1536; i++)
	{
		if( (image[i] & 0x40) == 0x40 )
		{
			begin = i;
			break;
		}
	}
	
	//image dont fits
	if( begin > 636 )
	{
		return;
	}
	
	for(uint8_t i=0; i<RESOLUTION; i++)
	{
		for(uint8_t j=0; j<RESOLUTION; j++)
		{
			char buff[4];
			//sprintf(buff,"%d",image[begin+i*30+j]);
			sprintf(buff,"%d",image[i*30+j]);
			uart1_puts(buff);
			if( j < 29 )
			{
				uart1_puts(";");
			}
		}
		uart1_puts("\r\n");
	}
	uart1_puts("-\r\n");
	
	
}

void printMotion(int8_t *motion)
{
	// 	uart1_puts("motion:\r\n");
	uart1_puts("x:");
	char moBuff[4];
 	sprintf(moBuff,"%d",motion[0]);
 	uart1_puts(moBuff);
	uart1_puts("\t");
	
	uart1_puts("y:");
	sprintf(moBuff,"%d",motion[1]);
	uart1_puts(moBuff);
	uart1_puts("\r\n");
}

void printMotionBurst(int8_t *burst)
{
	char moBuff[21];
	// sign of burst[1] and burst[2] switched to get motion from the perspective of the sensor
 	//sprintf(moBuff,"%d;%d;%d;%u\r\n",burst[0],burst[1]*(-1),burst[2]*(-1),burst[3]);
 	sprintf(moBuff,"%d;%d;%u\r\n",burst[1]*(-1),burst[2]*(-1),burst[3]);
 	uart1_puts(moBuff);
}

void printMotionBurstInRobot(int8_t *burst)
{
	char moBuff[21];
	// burst[1] and burst[2] switched to get motion from the perspective of the robot
 	sprintf(moBuff,"%d;%d;%d;%u\r\n",burst[0],burst[2],burst[1],burst[3]);
 	uart1_puts(moBuff);
}

void printPixelBuffer(uint8_t *image)
{
	for(uint16_t i=0; i<1536; i++)
	{
		char buff[4];
		sprintf(buff,"%d",image[i]);
		uart1_puts(buff);
		uart1_puts("\t");
	}
	uart1_puts("-------\r\n");
}

uint8_t read(uint8_t address)
{
	RESET(P_NCS);
	
	spi_exch(address);
	if (address == 0x02) // is motion read?
	{
		_delay_us(75);	// wait t_SRAD-MOT
	}
	else 
	{
		_delay_us(50);	// wait t_SRAD
	}
	uint8_t ret = spi_exch(0x00);
	SET(P_NCS);
	_delay_us(1);	// wait t_SRR = 250ns
	return ret;
}

void reset(void)
{
	SET(OF_RST);
	_delay_us(10);	// wait t_PW-RESET
	RESET(OF_RST);
	_delay_us(500);	// wait T_IN-RST
}

void setConfigurationBits(uint8_t conf)
{
	write(CONFIGURATION_BITS,conf);
}

void write(uint8_t address, uint8_t value)
{
	RESET(P_NCS);
	
	spi_exch(address|0x80);
	_delay_us(50);	// wait t_SWW / t_SWR
	spi_exch(value);
	SET(P_NCS);
}
