#include <avr/interrupt.h>
#include <avr/io.h>

/* ADC initialisieren */
void adc_init(void) {
 
	uint16_t result;
	ADMUX = (0<<REFS1) | (1<<REFS0);      // AVcc als Referenz benutzen
	//  ADMUX = (1<<REFS1) | (1<<REFS0);      // interne Referenzspannung nutzen
	//  ADMUX = (0<<REFS1) | (0<<REFS0);
	ADCSRA = (1<<ADPS1) | (1<<ADPS0);     // Frequenzvorteiler
	ADCSRA |= (1<<ADEN);                  // ADC aktivieren
 
	/* nach Aktivieren des ADC wird ein "Dummy-Readout" empfohlen, man liest
	also einen Wert und verwirft diesen, um den ADC "warmlaufen zu lassen" */
	
	ADCSRA |= (1<<ADSC);                  // eine ADC-Wandlung 
	while (ADCSRA & (1<<ADSC) ) {}        // auf Abschluss der Konvertierung warten
	/* ADCW muss einmal gelesen werden, sonst wird Ergebnis der nächsten
	Wandlung nicht übernommen. */
	result = ADCW;
}

/* ADC Einzelmessung */
uint16_t adc_read( uint8_t channel )
{
	// Kanal waehlen, ohne andere Bits zu beeinflußen
	ADMUX = (ADMUX & ~(0x1F)) | (channel & 0x1F);
	ADCSRA |= (1<<ADSC);            // eine Wandlung "single conversion"
	while (ADCSRA & (1<<ADSC) ) {}  // auf Abschluss der Konvertierung warten
	return ADCW;                    // ADC auslesen und zurückgeben
}

/* ADC Mehrfachmessung mit Mittelwertbbildung */
uint16_t adc_read_avg( uint8_t channel, uint8_t average )
{
	uint32_t result = 0;
 
	for (uint8_t i = 0; i < average; ++i )
		result += adc_read( channel );
	return (uint16_t)( result / average );
}
