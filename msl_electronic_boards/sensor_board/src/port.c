
#include <avr/io.h>
#include "port.h"

void initADC(void)
{
	unsigned int result;

	ADCSRA = (1 << ADEN) | (1 << ADPS0) | (1 << ADPS1) | (1 << ADPS2);
	ADMUX = (1<<REFS1) | (1<<REFS0);

	ADCSRA |= (1 << ADSC);

	while(ADCSRA & (1 << ADSC));

	result = ADCW;
}

/* ADC Einzelmessung */
uint16_t ADC_Read( uint8_t channel )
{
  // Kanal waehlen, ohne andere Bits zu beeinflußen
  ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
  ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
  while (ADCSRA & (1<<ADSC) ) {}  // auf Abschluss der Konvertierung warten
  return ADCW;                    // ADC auslesen und zurückgeben
}

/* ADC Mehrfachmessung mit Mittelwertbbildung */
uint16_t ADC_Read_Avg( uint8_t channel, uint8_t average )
{
  uint32_t result = 0;

  for (uint8_t i = 0; i < average; ++i )
    result += ADC_Read( channel );

  return (uint16_t)( result / average );
}
