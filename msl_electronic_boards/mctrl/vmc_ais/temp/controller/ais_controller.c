

#include "control/ais_controller.h"
#include "control/ais_configdata.h"
#include "ais_typedef.h"
#include "aisc167b/ais_gpt.h"
#include "system/ais_cmdbuff.h"
#include <stdio.h> 

struct CTRLSTATE ctrlState[NUM_MOTORS];
struct CTRLPARAM ctrlParam[NUM_MOTORS];


//////////////////////////////////////////////////////////////////////////////
/// \brief Set default controller parameter
//////////////////////////////////////////////////////////////////////////////
void defaultCtrlParameter() {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  TMC_UCHAR_8 channelID;
  for ( channelID = 0; channelID < NUM_MOTORS; channelID++ ) {
	  ctrlParam[channelID].nRamp  		= 5;
	  ctrlParam[channelID].pRamp		= 5;
	  ctrlParam[channelID].Kpr			= 0;
	  ctrlParam[channelID].Tn			= 0;
	  ctrlParam[channelID].Tv			= 0;
	  ctrlParam[channelID].deadband		= 10;
  }
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Initializes the controller
//////////////////////////////////////////////////////////////////////////////
void initController() {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  TMC_UCHAR_8 channelID;
  for ( channelID = 0; channelID < NUM_MOTORS; channelID++ ) {
	  ctrlState[channelID].sumI  			= 0;
	  ctrlState[channelID].oldSetRPM		= 0;
	  ctrlState[channelID].ctrlDesireRPM	= 0;
	  ctrlState[channelID].lastcyletime		= 0;
	  ctrlState[channelID].lastek  		 	= 0;
  }
}
//////////////////////////////////////////////////////////////////////////////

void ais_mctrl_reg_params(struct CmdStruct Command) {
  switch ( Command.cmd ) { 
    default: ;	
  }
}

//////////////////////////////////////////////////////////////////////////////
/// \brief Initializes the controller
//////////////////////////////////////////////////////////////////////////////
void resetController(TMC_UCHAR_8 channelID) {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  if ( (channelID >= 0) || (channelID < NUM_MOTORS) ) {
	  ctrlState[channelID].sumI = 0;
  }
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief the controller: PID with anti-windup and ramp function
// "real" PID in additiv form
//////////////////////////////////////////////////////////////////////////////
TMC_INT_16 controller(TMC_UCHAR_8 channelID, TMC_INT_16 desiredRPM,
					  TMC_INT_16 actualRPM, TMC_INT_16 stellerDiff,
					  TMC_INT_16 maxRPM, TMC_ULONG_32 last_cycle_length) {
// +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

	// temporary varibles ----------------------------------------------------
	TMC_LONG_32 ek  = 0; // controller deviation
	TMC_LONG_32 TA  = 0; // sample time
	TMC_LONG_32 ypk = 0; // proportional part
	TMC_LONG_32 yik = 0; // itegrativ part
	TMC_LONG_32 ydk = 0; // derativ part
	TMC_LONG_32 yrk = 0; // PID output sum
	TMC_LONG_32 lastek = 0; // deviation in last cycle
	TMC_LONG_32 actualTime = 0;
	TMC_INT_16  deltaDesire = 0;
	// -----------------------------------------------------------------------

	// Range check for channelID !!
	if ( (channelID < 0) || (channelID >= NUM_MOTORS) ) return 0;

	// Calculate difference to desired RPM in last cycle
	deltaDesire = desiredRPM - ctrlState[channelID].ctrlDesireRPM;

	// return last calculated value if no change in disired RPM
	if ( deltaDesire == 0 ) return ctrlState[channelID].oldSetRPM;

	// calculate ramp. If deltaDesire is bigger than max. allowed step
	// then reduce desiredRPM for this cycle.
	// Use apropriate ramp step (negativ ramp / positiv ramp)
	if ( deltaDesire > 0 ) { // use positiv Ramp
	  if ( ctrlParam[channelID].pRamp > 0 ) { // pRamp active ?
	   ctrlState[channelID].ctrlDesireRPM += MIN(ctrlParam[channelID].pRamp, deltaDesire);
	  } else { // Ramp not active
	   ctrlState[channelID].ctrlDesireRPM += deltaDesire; 
	  }
	} else { 				 // use negativ Ramp
	  if ( ctrlParam[channelID].nRamp > 0 ) { // pRamp active ?
	   ctrlState[channelID].ctrlDesireRPM += MAX(ctrlParam[channelID].nRamp, deltaDesire);
	  } else { // Ramp not active
	   ctrlState[channelID].ctrlDesireRPM += deltaDesire; 
	  }
	}

	// --- DEBUG INFORMATION ---------------------------------------------
	// now ctrlDesireRPM should be either desiredRPM or growing towards it
	// --- DEBUG INFORMATION ---------------------------------------------

	// calculate controller deviation
	ek 	   = ctrlState[channelID].ctrlDesireRPM - actualRPM;


	// for calculation of deravativ part we need also last deviation
	// get deviation in last cycle
	lastek = ctrlState[channelID].lastek;
	// sotre actual deviation for next cycle
	ctrlState[channelID].lastek = ek;

	// calculate sample time
	// get actual runtime since main loop start in us (to be checked!???)
	actualTime = get_cycle_time_part();
	// sample time is time since main-loop start plus time after last
	// controller call until end of loop :
	TA = (last_cycle_length - ctrlState[channelID].lastcyletime) + actualTime;
	// Converts to milliseconds
	TA = TA / 1000;

	// dead band: use controller only if deviation bigger than dead band
	// otherwise return last calculated value
	if ( ABS(ek) < ctrlParam[channelID].deadband ) return ctrlState[channelID].oldSetRPM;

	// START OF CONTROLLER ALGORYTHM -------------------------------------

	// Proportional part .................................................
	ypk = ctrlParam[channelID].Kpr * ek;

	// Integrativ part ...................................................
	// integrate with anti-windup:
	ctrlState[channelID].sumI += ek - stellerDiff;
	// calculate integrativ part
	if ( ctrlParam[channelID].Tn ) {
	 yik = ctrlParam[channelID].Kpr * (TA / ctrlParam[channelID].Tn);
 	 yik = yik * ctrlState[channelID].sumI;
    } else {
	 yik = 0;
	}

	// derativ part
	ydk = ctrlParam[channelID].Kpr * (ctrlParam[channelID].Tv / TA);
	ydk = ydk * (ek - lastek);

	// calculate PID sum
	yrk = ypk + yik + ydk;

	// check overflow of PID output yrk:
	yrk = MAX(yrk, maxRPM );
	yrk = MIN(yrk, -1 * maxRPM );

	// save calculated output for next cycle
	ctrlState[channelID].oldSetRPM = yrk;


	// return new controller output
	return (TMC_INT_16)yrk; //typecast long to int 16
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// Setter/Getter Functions for controller Parameters
//////////////////////////////////////////////////////////////////////////////


TMC_INT_16 ctrlgetnRamp(TMC_UCHAR_8 channelID) {
	if ( (channelID < 0) || (channelID >= NUM_MOTORS) ) return 0;
	return ctrlParam[channelID].nRamp;
}

void ctrlsetnRamp(TMC_UCHAR_8 channelID, TMC_INT_16 value) {
	if ( (channelID > 0) || (channelID < NUM_MOTORS) )
		ctrlParam[channelID].nRamp = value;
}

TMC_INT_16 ctrlgetpRamp(TMC_UCHAR_8 channelID) {
	if ( (channelID < 0) || (channelID >= NUM_MOTORS) ) return 0;
	return ctrlParam[channelID].pRamp;
}

void ctrlsetpRamp(TMC_UCHAR_8 channelID, TMC_INT_16 value) {
	if ( (channelID > 0) || (channelID < NUM_MOTORS) )
		ctrlParam[channelID].pRamp = value;
}

TMC_INT_16 ctrlgetdeadband(TMC_UCHAR_8 channelID) {
	if ( (channelID < 0) || (channelID >= NUM_MOTORS) ) return 0;
	return ctrlParam[channelID].deadband;
}

void ctrlsetdeadband(TMC_UCHAR_8 channelID, TMC_INT_16 value) {
	if ( (channelID > 0) || (channelID < NUM_MOTORS) )
		ctrlParam[channelID].deadband = value;
}

TMC_INT_16 ctrlgetKpr(TMC_UCHAR_8 channelID) {
	if ( (channelID < 0) || (channelID >= NUM_MOTORS) ) return 0;
	return ctrlParam[channelID].Kpr;
}

void ctrlsetKpr(TMC_UCHAR_8 channelID, TMC_INT_16 value) {
	if ( (channelID > 0) || (channelID < NUM_MOTORS) )
		ctrlParam[channelID].Kpr = value;
}

TMC_INT_16 ctrlgetTn(TMC_UCHAR_8 channelID) {
	if ( (channelID < 0) || (channelID >= NUM_MOTORS) ) return 0;
	return ctrlParam[channelID].Tn;
}

void ctrlsetTn(TMC_UCHAR_8 channelID, TMC_INT_16 value) {
	if ( (channelID > 0) || (channelID < NUM_MOTORS) )
		ctrlParam[channelID].Tn = value;
}

TMC_INT_16 ctrlgetTv(TMC_UCHAR_8 channelID) {
	if ( (channelID < 0) || (channelID >= NUM_MOTORS) ) return 0;
	return ctrlParam[channelID].Tv;
}

void ctrlsetTv(TMC_UCHAR_8 channelID, TMC_INT_16 value) {
	if ( (channelID > 0) || (channelID < NUM_MOTORS) )
		ctrlParam[channelID].Tv = value;
}
