//////////////////////////////////////////////////////////////////////////////
/// \ingroup eeprom EEPROM
//@{
/// \file ais_eeprom.h
///
/// \brief 	Header File for eeprom Module
///
/// \author Pascal Langenberg
/// \version 0.1
///
/// \date 9.08.2006
///
/// \note Edit this File to change Timeouts or Change I/O Pins for I2C Bus
///
//////////////////////////////////////////////////////////////////////////////

#ifndef _AIS_EEPROM_H_
#define _AIS_EEPROM_H_

#include "ais_typedef.h"

VMC_UCHAR_8 i2c_storeByte(VMC_UCHAR_8 data, VMC_UINT_16 address);
VMC_UCHAR_8 i2c_loadByte(VMC_UINT_16 address);
VMC_UCHAR_8 i2c_storeInt(VMC_UINT_16 data, VMC_UINT_16 address);
VMC_INT_16 i2c_loadInt(VMC_UINT_16 address);
void i2c_read_eeprom(void);
void i2c_erase_eeprom(void);

#endif /* _AIS_EEPROM_H_ */

//@}