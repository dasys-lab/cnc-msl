
#include "vision.h"
uint32_t timeLastSended = 0;
#define VISION_TIMEOUT	500 //ms

void vision_init(void){
	SET_INPUT(VISION);
	SET(VISION);	

	SET_OUTPUT(VISION_LED);
	RESET(VISION_LED);

	/*uint8_t id[4] = {0x00, PRIORITY_NORM, SENSORBOARD_ID, ETH2CAN_ID};
	generate_extCAN_ID(id, message.id);
	message.header.rtr = 0;
	message.header.length = 1;*/
}

void	visionSendStatus(uint32_t time){
	if ( IS_SET(VISION) &&  (time-timeLastSended) > VISION_TIMEOUT ) {
		can_put_cmd(CMD_VISION_STATE,"",1);
		timeLastSended = time;
	}
	
	return;
}
