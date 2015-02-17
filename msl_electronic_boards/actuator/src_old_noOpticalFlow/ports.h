#ifndef PORTS_H
#define PORTS_H

void ports_init(void);
void toggle_status_led(void);
void select_adc_channel(uint8_t mux);
uint16_t get_capacitors_voltage(void);
float get_supply_voltage(void);
uint16_t get_supply_raw_voltage(void);
uint16_t read_adc(void);

#endif
