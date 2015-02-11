
#include "button.h"

void button_init(void){
	SET_INPUT(BUTTON);
	SET(BUTTON);
	
	SET_OUTPUT(BUTTON_LED);
	RESET(BUTTON_LED);
	/*uint8_t id[4] = {0x00, PRIORITY_NORM, SENSORBOARD_ID, ETH2CAN_ID};
	generate_extCAN_ID(id, message.id);
	message.header.rtr = 0;
	message.header.length = 1;*/
}

void	buttonSendStatus(uint32_t time){
	
	return;
}
