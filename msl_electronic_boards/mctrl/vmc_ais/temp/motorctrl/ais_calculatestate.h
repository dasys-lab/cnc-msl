//////////////////////////////////////////////////////////////////////////////
/// \ingroup calculatestate
//@{
/// \file ais_calculatestate.h
///
/// \brief Header file for the calculate module
/// 
/// \author Pascal Langenberg
///
/// \version 0.1
///
/// \date 15.05.2006
///
//////////////////////////////////////////////////////////////////////////////

#ifndef AIS_CALCULATE_H
#define AIS_CALCULATE_H

#include "ais_typedef.h"

void init_Calculatestate();
TMC_INT_16 calc_Encoderticks(TMC_UCHAR_8 id);
TMC_LONG_32 calc_RPM(TMC_ULONG_32 last_cycle_time, struct MOTORSTATE *motorstate);
TMC_LONG_32 calc_abs_Rots(struct MOTORSTATE *motorstate);
TMC_UINT_16 calc_Voltage(TMC_UCHAR_8 id);
TMC_UINT_16 calc_Current(TMC_UCHAR_8 id);
TMC_UINT_16 calc_Temperature(TMC_ULONG_32 last_cycle_time,  struct MOTORSTATE *motorstate);
TMC_UINT_16 calc_Torque(struct MOTORSTATE *motorstate);
TMC_UINT_16 calc_Power(struct MOTORSTATE *motorstate);



#endif /* AIS_CALCULATE_H */

//@}