#include "mcp2515.h"
#include "global.h"
#include "defaults.h"


int main(void) {
	tExtendedCAN message, m;
	tCAN reply;
	int i=0;
	
	mcp2515_init();
		
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);

	message.id[0] = 0x83;
	message.id[1] = 0xFF;
	message.id[2] = 0xFF;
	message.id[3] = 0x01;
	
	message.header.rtr = 0;
	message.header.length = 5;
	message.data[0] = 'H';
	message.data[1] = 'A';
	message.data[2] = 'L';
	message.data[3] = 'L';
	message.data[4] = 'O';
	
	mcp2515_send_extmessage(&message);	 
	
	while(1) {
		if(mcp2515_check_message()) {
			mcp2515_get_extmessage(&message);
		        mcp2515_send_extmessage(&message);
		}
		++i;
		//if(i==0) mcp2515_send_extmessage(&message);
//		mcp2515_send_message(&message);
	}
	return 0;
}
