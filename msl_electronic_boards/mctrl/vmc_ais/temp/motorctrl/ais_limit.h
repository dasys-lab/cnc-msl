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


#ifndef AIS_LIMIT_H
#define AIS_LIMIT_H

#define ABSOLUTEMAX_CURRENT 8000

void init_Limiter(unsigned char ID, unsigned int tn);
unsigned char get_Limiter_state(unsigned char ID);
int CurrentLimitter(unsigned char Motor_ID, int inValue, int max_current, int actual_current, long DeltaTIME);


#endif //AIS_LIMIT_H
//@}
