//////////////////////////////////////////////////////////////////////////////
/// \defgroup limit current limiter
//@{
/// \file ais_limiter.c
///
/// \brief Current limiter function
///
/// \note all Currents are in milli Ampere and positiv at all times
///
/// \author Adam Cwientzek
///
/// \version 0.1
///
/// \date 09.10.2006
///
//////////////////////////////////////////////////////////////////////////////

#include "motorctrl/ais_limiter.h"

void init_current_limiter() {
}

unsigned char get_limiter_state() {
 return 0;
}

VMC_INT_16 current_limiter(VMC_INT_16 pwm) {
 return pwm;
}

//@}


