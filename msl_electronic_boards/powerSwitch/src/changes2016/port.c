
#include <avr/io.h>
#include "port.h"

uint16_t read_adc(void) {

        uint8_t i;
        uint16_t res = 0;

        // wait for end of last (test-) conversion
        while (ADCSRA & (1 << ADSC));

        // read channel
        for (i = 0; i < 3; i++) {
                ADCSRA |= (1 << ADSC);
                while (ADCSRA & (1 << ADSC));
                res += ADCW;
        }
        res /= 3;

        return res;
}

void select_adc_channel(uint8_t mux) {

        uint8_t last_channel, new_channel;

        last_channel = ADMUX;
        new_channel = (1 << REFS0) | mux;

        // change channel
        if (last_channel != new_channel) {
                ADMUX = new_channel;
                // start a test-conversion, but do not wait
                ADCSRA |= (1 << ADSC);
        }

        return;
}
