//////////////////////////////////////////////////////////////////////////////
/// \ingroup controller
//@{
/// \file ais_controller.h
///
/// \brief Header file for the Controller Modul
///
/// \author Adam Cwientzek
///
/// \version 0.5
///
/// \date 2.10.2006
///
//////////////////////////////////////////////////////////////////////////////

#ifndef AIS_CONTROLLER_H
#define AIS_CONTROLLER_H

#include "ais_typedef.h"

#define ABS(a)   ( ((a) > 0) ? (a) : -(a) )

#define MAX(a,b) ( ((a) > (b)) ? (a) : (b) )
#define MIN(a,b) ( ((a) < (b)) ? (a) : (b) )

struct CTRLSTATE {
    TMC_LONG_32 sumI;
    TMC_LONG_32 oldSetRPM;
    TMC_LONG_32 ctrlDesireRPM;
    TMC_LONG_32 lastcyletime;
	TMC_LONG_32 lastek;
};

struct CTRLPARAM {
    TMC_INT_16 nRamp, pRamp;
    TMC_INT_16 deadband;
    TMC_INT_16 Kpr;
    TMC_INT_16 Tn;
    TMC_INT_16 Tv;
};


void defaultCtrlParameter();
void initController();
void ais_mctrl_reg_params(struct CmdStruct Command);

void resetController(TMC_UCHAR_8 channelID);
TMC_INT_16 controller(TMC_UCHAR_8 channelID, TMC_INT_16 desiredRPM,
					  TMC_INT_16 actualRPM, TMC_INT_16 stellerDiff,
					  TMC_INT_16 maxRPM, TMC_ULONG_32 last_cycle_length);

TMC_INT_16 ctrlgetnRamp(TMC_UCHAR_8 channelID);
void ctrlsetnRamp(TMC_UCHAR_8 channelID, TMC_INT_16 value);
TMC_INT_16 ctrlgetpRamp(TMC_UCHAR_8 channelID);
void ctrlsetpRamp(TMC_UCHAR_8 channelID, TMC_INT_16 value);
TMC_INT_16 ctrlgetdeadband(TMC_UCHAR_8 channelID);
void ctrlsetdeadband(TMC_UCHAR_8 channelID, TMC_INT_16 value);
TMC_INT_16 ctrlgetKpr(TMC_UCHAR_8 channelID);
void ctrlsetKpr(TMC_UCHAR_8 channelID, TMC_INT_16 value);
TMC_INT_16 ctrlgetTn(TMC_UCHAR_8 channelID);
void ctrlsetTn(TMC_UCHAR_8 channelID, TMC_INT_16 value);
TMC_INT_16 ctrlgetTv(TMC_UCHAR_8 channelID);
void ctrlsetTv(TMC_UCHAR_8 channelID, TMC_INT_16 value);



#endif /* AIS_CONTROL_H */

//@}
