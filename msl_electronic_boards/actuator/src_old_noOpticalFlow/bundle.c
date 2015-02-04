
#include "bundle.h"
uint32_t timeBundleLastSended = 0;
#define BUNDLE_TIMEOUT	500 //ms

void bundle_init(void){
	SET_INPUT(BUNDLE);
	SET(BUNDLE);

	SET_OUTPUT(BUNDLE_LED);
	RESET(BUNDLE_LED);
	
	/*uint8_t id[4] = {0x00, PRIORITY_NORM, SENSORBOARD_ID, ETH2CAN_ID};
	generate_extCAN_ID(id, message.id);
	message.header.rtr = 0;
	message.header.length = 1;*/
}

void	bundleSendStatus(uint32_t time){
	if ( IS_SET(BUNDLE) &&  (time-timeBundleLastSended) > BUNDLE_TIMEOUT ) {
		can_put_cmd(CMD_BUNDLE_STATE,"",1);
		timeBundleLastSended = time;
	}
	
	return;
}
