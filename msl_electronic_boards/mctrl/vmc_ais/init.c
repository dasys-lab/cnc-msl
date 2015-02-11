//////////////////////////////////////////////////////////////////////////////
/// \file init.c
///
/// \brief Initialization for Custom Program
/// 
/// \author Jan Paulus, Adam Cwientzek and Pascal Langenberg
///
/// \version 0.9
///
/// \date 9.08.2006
///
//////////////////////////////////////////////////////////////////////////////

  
#include "init.h"

#include "comasc/ais_asccom.h"
#include "aisc167b/ais_gpt.h"
#include "aisc167b/ais_pwm.h"
#include "aisc167b/ais_i2c.h"
#include "aisc167b/ais_bioport.h"

#include "system/ais_astring.h"
#include "system/ais_utils.h"  // to be changed !
#include "system/ais_system.h"

#include "mpwr/ais_mpwr.h"
#include "aisc167b/ais_led.h"

#include "motorctrl/ais_configmanager.h"
#include "motorctrl/ais_motorconfig.h"
#include "motorctrl/ais_motorctrl.h"
#include "motorctrl/ais_calculatestate.h"
#include "motorctrl/ais_error.h"
#include "motorctrl/ais_limiter.h"

#include "controller/ais_controller.h"
#include "aisc167b/ais_bioport.h"
#include "can_stuff/ais_can.h"


//////////////////////////////////////////////////////////////////////////////
/// Inizialization. This function is called once before entering 
/// main loop. 
///
//////////////////////////////////////////////////////////////////////////////
void init() {
	VMC_INT_16 motorID;

	init_aisc167b();
    init_asccom();    
    init_cycletime_counter();
	i2c_init();
	init_bioport();
	if (can_init(CAN_BAUD, 0)) while(1){led_swap_green();}

    led_set_green(0);
    led_set_red(0);
	ledseq_init();

    ais_system_init();


	//init_Thermic();
	init_error();
	init_calculatestate();
	init_motorcontrol();
	init_controller();
	init_current_limiter();
	mpwr_init();

	set_default_configuration();

	for (motorID = 0; motorID < 3; motorID++) {
	//	init_Limiter(i, 100);
		init_encoder(motorID);
	}
   


}
//////////////////////////////////////////////////////////////////////////////


