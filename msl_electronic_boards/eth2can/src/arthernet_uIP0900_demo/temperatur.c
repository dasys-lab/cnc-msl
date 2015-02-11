#include"temperatur.h"
#include "onewire.h" 
#include "ds18x20.h" 
#include "delay.h" 

#define MAXSENSORS 1 

uint8_t gSensorIDs[MAXSENSORS][OW_ROMCODE_SIZE]; 
uint8_t nSensors; 

uint8_t search_sensors(void); 
uint8_t subzero, cel, cel_frac_bits;

uint8_t search_sensors(void) 
{ 
    uint8_t i; 
    uint8_t id[OW_ROMCODE_SIZE]; 
    uint8_t diff, nSensors; 

    //uart_puts_P( "\rScanning Bus for DS18X20\r" ); 

    nSensors = 0; 

    for( diff = OW_SEARCH_FIRST; 
         diff != OW_LAST_DEVICE && nSensors < MAXSENSORS ; ) 
    { 
        DS18X20_find_sensor( &diff, &id[0] ); 

        if( diff == OW_PRESENCE_ERR ) { 
            //uart_puts_P( "No Sensor found\r" ); 
            break; 
        } 

        if( diff == OW_DATA_ERR ) { 
            //uart_puts_P( "Bus Error\r" ); 
            break; 
        } 

        for (i=0;i<OW_ROMCODE_SIZE;i++) 
            gSensorIDs[nSensors][i]=id[i]; 

        nSensors++; 
    } 

    return nSensors; 
}

void init_sensors(void){
	
	nSensors = search_sensors(); 
}

/* mehrere sensoren zu lesen ist natürlich quatsch wenn man
*  a) nur einen hat :-)
*  b) die Variablen mit dem Inhalt anderer Sensoren überschreibt :-)
*
*  That's Guido Source - programming is colorful
*/

void read_temp_sensors(void){

	unsigned int i = 0;
	
	if ( DS18X20_start_meas( DS18X20_POWER_PARASITE, NULL ) == DS18X20_OK) { 
	
		delay_ms(DS18B20_TCONV_12BIT); 
		
		for ( i=0; i<nSensors; i++ ){ 
			
			if ( DS18X20_read_meas( &gSensorIDs[i][0], &subzero, &cel, &cel_frac_bits)
				== DS18X20_OK ) 
			{ 
				cel_frac_bits = cel_frac_bits *DS18X20_FRACCONV;
			}
			
		} 
	} 

}

