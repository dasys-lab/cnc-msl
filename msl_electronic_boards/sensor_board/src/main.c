#define F_CPU 16000000UL

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>

#include "uart.h"
#include "srf08.h"
#include "util.h"
#include "twimaster.h"
#include "lis3l.h"
#include "cap.h"
#include "spi.h"
#include "port.h"
#include "mlx90609.h"

#define VERSION "sensor v1.5\n"
#define SRF08 0x31
#define LIS3L 0x32
#define CAP 0x33
#define GYRO 0x34
#define INFRA 0x35
#define INTR 0x36

char buf[BUFFERLENGTH];

void mainInterrupt(void)
{
	unsigned int result;

	DDRA &= ~(1 << DDA7);
	PORTA |= (1 << PA7);

	while((PINA & (1<<PA7)) == 0)

	uart1_puts("nach busy waiting\n");

	SPI_Init();

	mlx90609_init();

	mlx90609_mode(ST_NONE);

	//timer starten

	TCNT1H=0;  //  Timer Daten Register Timer1 High auf 0 Setzen
	TCNT1L = 0; //  Timer Daten Register Timer1 Low auf 0 Setzen

	TCCR1B |= (1<<CS12); // Timer Starten mit Vorteiler 256

	//insert get code here

	mlx90609_adcstart(CH_GYRO);
	result = mlx90609_adcread();

	uart1_puts("nach read\n");

	result = TCNT1;

	sprintf(buf, "\n%u", result);

	uart1_puts(buf);

	while(1);

	//senden
}

void mainInfra(void)
{
	initADC();

	while(1)
	{
		int w = ADC_Read_Avg(1, 3);
		w = 5000.0 / 1024.0 * w;
		sprintf(buf,"%04d\n", w);
		uart1_puts(buf);
	}
}

void mainGyro(void)
{
	unsigned int result;

	SPI_Init();

	mlx90609_init();

	mlx90609_mode(ST_NONE);

	while(1)
	{

		mlx90609_adcstart(CH_GYRO);

		uart1_puts("nach adc start\n");

		result = mlx90609_adcread();

		uart1_puts("nach read\n");

		sprintf(buf,"%u\n", result);

		uart1_puts(buf);

		mlx90609_adcstart(CH_TEMP);

		result = mlx90609_adcread();

		result = ((25.0/16.0 * result) + 300.0) / 100.0;

		sprintf(buf, "temp: %u\n", result);

		uart1_puts(buf);
	}
}

void mainCap(void)
{
	initCap();

	long value;
	double realValue;

	while(1)
	{
		value = /*(0.000000224 * 0x800000) -  *//*(0.164 * */read()/*)*/;

		realValue = (0.000000224 * 0x800000) - (0.164 * (double) value);
		//realValue = realValue * 1000000.0;
		sprintf(buf, "%d\n", (long) realValue);
		uart1_puts(buf);
	}
}

void mainSRF08(void)
{
	initSRF();

	while(1)
	{
		if(uart1_data_received())
		{
			unsigned char c = (unsigned char) uart1_getc();

			if(c == AMP_COMMAND)
			{
				setAmpValue();
			}

			else if(c == RANGE_COMMAND)
			{
				uart1_puts("range set\n");
				setRangeValue();
			}

			else if(c == AUTOCAL_COMMAND)
			{
				autoCalibration();
				initSRF();
			}
		}

		else
		{
			unsigned int result = getRange();
			printValue(result);
		}
	}
}

void mainLIS3L(void)
{
	initLis3l();

	while(1)
	{
		sprintf(buf, "%i\t%i\t%i\n", readX(), readY(), readZ());
		uart1_puts(buf);
	}
}

int main(void)
{
	i2c_init();

	uart1_init(UART_BAUD_SELECT(9600, F_CPU));
	sei();

	uart1_puts(VERSION);

	while(1)
	{
		uart1_puts("1. Ultraschallsensor\n2. Beschleunigungssensor\n3. Kapazitiver Sensor\n4. Gyroskop\n5. Infrarot\n6. Interrupt");

		while(!uart1_data_received());

		unsigned char c = (unsigned char) uart1_getc();

		if(c == SRF08)
		{
			mainSRF08();
		}

		if(c == LIS3L)
		{
			mainLIS3L();
		}

		if(c == CAP)
		{
			mainCap();
		}
		
		if(c == GYRO)
		{
			mainGyro();
		}

		if(c == INFRA)
		{
			mainInfra();
		}

		if(c == INTR)
		{
			mainInterrupt();
		}

	}

	return 0;
}

