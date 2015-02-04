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

void init_calculatestate();
VMC_INT_16 calc_encoderticks(VMC_UCHAR_8 id);
VMC_LONG_32 calc_RPM(TMC_ULONG_32 last_cycle_length, struct MOTORSTATE *motorstate, VMC_UINT_16 ticks_per_Rot);
VMC_LONG_32 calc_abs_rots(struct MOTORSTATE *motorstate, VMC_UINT_16 ticks_per_rot);
VMC_UINT_16 calc_voltage(VMC_UCHAR_8 id);
VMC_UINT_16 calc_current(VMC_UCHAR_8 id);
VMC_UINT_16 calc_temperature(TMC_ULONG_32 last_cycle_time,  struct MOTORSTATE *motorstate);
VMC_UINT_16 calc_torque(struct MOTORSTATE *motorstate, VMC_UINT_16 spec_torque);
VMC_UINT_16 calc_power(struct MOTORSTATE *motorstate);


#endif /* AIS_CALCULATE_H */

//@}