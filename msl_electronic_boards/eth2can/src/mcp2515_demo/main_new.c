#include "mcp2515.h"
#include "global.h"
#include "defaults.h"


int main(void) {
	tCAN message;
	
	mcp2515_init();
		
	mcp2515_bit_modify(CANCTRL, (1<<REQOP2)|(1<<REQOP1)|(1<<REQOP0), 0);

	message.id = 0x123;
	message.header.rtr = 0;
	message.header.length = 2;
	message.data[0] = 0xab;
	message.data[1] = 0xcd;
		 
	while(1) mcp2515_send_message(&message);
	return 0;
}
