
#include "lightbarrier.h"
uint8_t haveBallState = 7;
uint32_t timeHaveBall = 0;
#define HAVE_BALL_TIMEOUT	500 //ms

void lightbarrier_init(void){
	SET_INPUT(LIGHT_BARRIER);
	SET(LIGHT_BARRIER);
	
	/*uint8_t id[4] = {0x00, PRIORITY_NORM, SENSORBOARD_ID, ETH2CAN_ID};
	generate_extCAN_ID(id, message.id);
	message.header.rtr = 0;
	message.header.length = 1;*/
}

void	sendStatus(uint32_t time){
	uint8_t tmp = 7;
	if ( !IS_SET(LIGHT_BARRIER) ) {
		tmp = 0;
	} else {
		tmp = 1;
	}
	
	if( tmp != haveBallState ||
		(timeHaveBall == 0 || (time-timeHaveBall) > HAVE_BALL_TIMEOUT)
	)
	{
		timeHaveBall = time;
		haveBallState = tmp;
		//message.data[0] = haveBallState;
		//can_send_message(&message);
		can_put_cmd(CMD_HAVE_BALL,&haveBallState,1);
	}
	return;
}
