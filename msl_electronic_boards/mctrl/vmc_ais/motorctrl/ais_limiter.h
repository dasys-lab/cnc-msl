//////////////////////////////////////////////////////////////////////////////
/// \ingroup limit Current Limiter
//@{
/// \file ais_limit.h
///
/// \brief Header File for the Current Limiter function
///
///
///
/// \author Jan Paulus
///
/// \version 0.8
///
/// \date 08.08.2005
///
//////////////////////////////////////////////////////////////////////////////


#ifndef AIS_LIMITER_H
#define AIS_LIMITER_H

#define MAX_CURRENT 8000

#include "ais_typedef.h"

void init_current_limiter();
unsigned char get_current_limiter_state();
VMC_INT_16 current_limiter(VMC_INT_16 pwm);


#endif //AIS_LIMITER_H
//@}
