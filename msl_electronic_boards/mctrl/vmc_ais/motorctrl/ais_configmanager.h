//////////////////////////////////////////////////////////////////////////////
/// \ingroup configsave Configsave
//@{
/// \file ais_configmanager.h
///
/// \brief 	Header File for Confuguration save/load module
///
/// \author Pascal Langenberg, Adam Cwientzek
///
/// \version 0.2
///
/// \date 03.10.2006
///
/// \note
///
//////////////////////////////////////////////////////////////////////////////
#ifndef _AIS_CONFIGMANAGER_H_
#define _AIS_CONFIGMANAGER_H_

#include "ais_typedef.h"

//Offsets of motorindependent parameters, add 0x0002 to last address and define this address for new parameters


#define _ADDR_FLAG_ 			0x00
#define _ADDR_TIMEOUT_ 			0x02


//Offets for motor parameters
#define _ADDR_MAX_CURRENT_		0x01
#define _ADDR_MAX_RPM_			0x02
#define _ADDR_NOM_CURRENT_		0x03 
#define _ADDR_NOM_RPM_			0x04
#define _ADDR_GEAR_REDUCTION_	0x05
#define _ADDR_WHEEL_RADIUS_		0x06
#define _ADDR_TICKS_ROT_		0x07
#define _ADDR_SPEC_TORQUE_		0x08
#define _ADDR_MAX_TEMP_			0x09
#define _ADDR_NOM_TEMP_			0x0a
#define _ADDR_WINDING_T_	    0x0b
#define _ADDR_WINDING_G_Z_	    0x0c
#define _ADDR_WINDING_G_N_		0x0d
#define _ADDR_CHASSIS_T_	    0x0e
#define _ADDR_CHASSIS_G_Z_		0x0f 
#define _ADDR_CHASSIS_G_N_		0x10

// Controller Parameters
#define _ADDR_KP_				0x20
#define _ADDR_TN_				0x21
#define _ADDR_TV_				0x22
#define _ADDR_DEADBAND_			0x23
#define _ADDR_NRAMP_			0x24
#define _ADDR_PRAMP_			0x25

#define _ADDR_DIRECTION_		0x2a
#define _ADDR_CONTROLLER_		0x2b
#define _ADDR_USEPWM_			0x2c
#define _ADDR_LIMITER_			0x2d




//Offsets of the saving areas of the motors
#define _OFFSET_MOTOR_ 			0x0080

//offsets for the base adresses of the different configuarations
#define _OFFSET_CONFIG_ 		0x0200

// Offset in EEPROM where to start first configuration
#define _OFFSET_BASE_			0x0F00



void set_default_configuration();
VMC_UCHAR_8 save_configuration(VMC_UCHAR_8 config);
VMC_UCHAR_8 load_configuration(VMC_UCHAR_8 config);

//adress = config + motor + parameter
VMC_UCHAR_8 ic2_storeByte(VMC_UCHAR_8 data, VMC_UINT_16 address);
VMC_UCHAR_8 ic2_loadByte(VMC_UINT_16 address);
VMC_UCHAR_8 ic2_storeInt(VMC_UINT_16 data, VMC_UINT_16 address);
VMC_INT_16  ic2_loadInt(VMC_UINT_16 address);




#endif /* _AIS_CONFIGMANAGER_H_ */

//@}
