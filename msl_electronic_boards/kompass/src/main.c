#include <util/delay.h>
#include <avr/io.h>
#include "mcp2515.h"
#include "cmps03.h"
//#include "cmpsHelper.h"
//#include "messages.h"

#define COMPASS_ID		0x40

volatile uint16_t ms_count;
int main(void) {
	tExtendedCAN message;
	tExtendedCAN reply;

	//vars for the compass values
	int16_t d;
	TWSR = 0;
	TWBR = 28;

	mcp2515_init();

	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);

	// this is the hardcoded id: 0x00404000
	message.id[0] = 0x02;
	message.id[1] = 0x0C;
	message.id[2] = 0x40;
	message.id[3] = 0x00;
	
	message.header.rtr = 0;

	while(1){

		// hole nachricht

		if (mcp2515_check_message()) {
			mcp2515_get_extmessage(&reply);
			// soft-CAN_ID-filter
			if (reply.id[3] == COMPASS_ID) {
				//char str[4];

				//calibrate
				if (reply.data[0] == 'c') {
					
					cmps03_calibrate();
					//sprintf(str, "Cal\n");
					//debug(str);
				}
				//else {
					//error("ERR IMPL");
				//}
			}

		}

		d=bearing16();
		message.header.length = 2;
		message.data[0] = (uint8_t) (d>>8);
		message.data[1] = (uint8_t) d;
		mcp2515_send_extmessage(&message);
		
		_delay_us(100000);
	}
	return 0;
}

