/* ADC initialisieren */
void adc_init(void);

/* ADC Einzelmessung */
uint16_t adc_read( uint8_t channel );

/* ADC Mehrfachmessung mit Mittelwertbbildung */
uint16_t adc_read_avg( uint8_t channel, uint8_t average );

