//////////////////////////////////////////////////////////////////////////////
/// \ingroup thermic Thermic Modul
//@{
/// \file ais_thermic.h
///
/// \brief Header file for the Thermic Modul
/// 
/// \author Jan Paulus and Dipl. Ing. Kubina
///
/// \version 0.8
///
/// \date 10.08.2005
///
//////////////////////////////////////////////////////////////////////////////

#ifndef THERMIC_H
#define THERMIC_H

#include "ais_typedef.h"

void init_Thermic(void);
TMC_UINT_16 getMotorWindingTemperature(TMC_UCHAR_8 Motor_ID);
TMC_UINT_16 getMotorChassisTemperature(TMC_UCHAR_8 Motor_ID);
void setEnvironment_Temperature(const TMC_UINT_16 ENV_TEMP);
void set_WindingConstants(TMC_UCHAR_8 Motor_ID, TMC_UINT_16 T, TMC_UINT_16 G_N, TMC_UINT_16 G_Z);
void set_ChassisConstants(TMC_UCHAR_8 Motor_ID, TMC_UINT_16 T, TMC_UINT_16 G_N, TMC_UINT_16 G_Z);
TMC_UINT_16 calcMotorTemperature(const TMC_UCHAR_8 Motor_ID, TMC_UINT_16 actCurrent, TMC_UINT_16 DeltaTIME);
void setThermicHysteresisSwitchConstants( TMC_UCHAR_8 Motor_ID, TMC_UINT_16 MIN_Temperatur, TMC_UINT_16 MAX_Temperatur);
void setCurrentValues( TMC_UCHAR_8 Motor_ID, TMC_UINT_16 c_nominal, TMC_UINT_16 c_max);
TMC_UINT_16 calcThermicHysteresisSwitch( TMC_UCHAR_8 Motor_ID, TMC_UINT_16 actValue);
TMC_UCHAR_8 getThermicHysteresisSwitchState( TMC_UCHAR_8 Motor_ID);
void Thermic( TMC_UCHAR_8 Motor_ID, TMC_UINT_16 actual_current, TMC_UINT_16* max_current, TMC_UINT_16* actual_temp, const TMC_UINT_16 DeltaTIME );

#endif //THERMIC_H
