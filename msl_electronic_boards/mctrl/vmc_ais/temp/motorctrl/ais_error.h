//////////////////////////////////////////////////////////////////////////////
/// \ingroup error
//@{
/// \file ais_error.h
///
/// \brief Header file for the errorchecker
///
/// \author Pascal Langenberg
///
/// \version 0.1
///
/// \date 15.05.2006
///
//////////////////////////////////////////////////////////////////////////////

#ifndef AIS_ERROR_H
#define AIS_ERROR_H

#include "ais_typedef.h"

/// Structure for Error_check flags
struct ERR_MOTOR  {
	 /// flag if there is a mechanical problem with the motor 0: no problem 1: problem
     TMC_UCHAR_8 motor;
	 /// flag if there is a problem with the encodersignals 0: no problem 1: problem
     TMC_UCHAR_8 encoder;
};

/// Structure representing the state of the controller
struct ERR_STATES {
   /// flag if timeout has appears 0: no timeout 1:timeout
   TMC_UCHAR_8 timeout;
   /// flag if an emergencystop is necassary 0: no ES necessary 1: ES necessary
   TMC_UCHAR_8 emergencystop;
   struct ERR_MOTOR err_motor[NUM_MOTORS];
};


void init_Error();
TMC_UCHAR_8 chk_Motor(TMC_UCHAR_8 id);
TMC_UCHAR_8 chk_Timeout(TMC_LONG_32 last_cycle_length);
TMC_UCHAR_8 chk_Encoder(TMC_UCHAR_8 id);
TMC_UCHAR_8 chk_Emergencystop(struct ERR_STATES *error_sys);
void reset_Timeout();


#endif /* AIS_ERROR_H */

//@}





