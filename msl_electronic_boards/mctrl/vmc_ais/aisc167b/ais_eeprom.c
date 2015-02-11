//////////////////////////////////////////////////////////////////////////////
/// \defgroup eeprom Module for using the eeprom
//@{
/// \file ais_eeprom.c
///
/// \brief 	funtions for reading and writing data to the connected eeprom via i2c
///
/// \author Pascal Langenberg
///
/// \version 0.1
///
/// \date 09.08.2006
///
/// \note To Change Pins for I2C Bus edit the file ic2.h
///
//////////////////////////////////////////////////////////////////////////////

#include "aisc167b/ais_eeprom.h"
#include "aisc167b/ais_i2c.h"
 
// only for printf
#include "comasc/ais_ascbuff.h"
#include <stdio.h>

//////////////////////////////////////////////////////////////////////////////
/// \brief Stores an Integer value into the eeprom via i2c
/// \param data Integer value to save
/// \param address The Target address
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2c_storeInt(VMC_UINT_16 data, VMC_UINT_16 address) {

	// local chars for the data
	// LSB to datalow
 	VMC_UCHAR_8 datalow = data;
	// MSB to datahigh
	VMC_UCHAR_8 datahigh = data >> 8;

	// Write first char, MSB to the given address
	if (i2c_storeByte(datahigh, address) != 0) return 1;

	// Wait until I2c is ready again
	i2c_delay(20000);

	// Write second char, LSB to the next address
	if (i2c_storeByte(datalow, (address + 1)) != 0) return 1;
	
	// Wait until I2c is ready again
	i2c_delay(10000);

	//return 0: succesfull write
	//printf("ADDR: %d \t value: %d \t LSB: %d \t MSB: %d \n", address, data, datalow, datahigh);
	return 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
/// \brief Reurns an Integer value saved ind the eeprom via i2c
/// \param address The Target address
/// \returns Integer Value at the address
//////////////////////////////////////////////////////////////////////////////
VMC_INT_16 i2c_loadInt(VMC_UINT_16 address) {

	// local chars to save the readed bytes

	//LSB
    VMC_UCHAR_8 datalow;

	//MSB
	VMC_UCHAR_8 datahigh;

	//Read MSB from i2c
	datahigh = i2c_loadByte(address);

	// Wait until I2c is ready again
	i2c_delay(20000);

	// Read LSB from i2c
	datalow = i2c_loadByte(address + 1);

	// Wait until I2c is ready again
	i2c_delay(20000);

	//return data as one Integer
	//printf("ADDR: %d \t value: %d \t LSB: %d \t MSB: %d \n", address, datahigh * 256 + datalow, datalow, datahigh);
	return datahigh * 256 + datalow;
}
//////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////
/// \brief Write a byte into the eeprom via i2c
/// \param Target Adress of the write operation
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2c_storeByte(VMC_UCHAR_8 data, VMC_UINT_16 address) {

	// local chars for the MSB/LSB of the address. i2c supports just 8bit words, eeprom Address are 16 bit long
	//LSB of the address
	VMC_UCHAR_8 addrlow = address;
	//MSB of the address
    VMC_UCHAR_8 addrhigh = address >> 8;

	// Send start condition to i2c
	i2c_start();

	// Send device select code
	if (i2c_sendByte(0xa0) != 0) return 1;
	i2c_delay(50);

	// Send address MSB
	if (i2c_sendByte(addrhigh) != 0) return 1;
	i2c_delay(50);

	// Send address LSB
    if (i2c_sendByte(addrlow) != 0)  return 1;
	i2c_delay(100);

	// Send data
	if (i2c_sendByte(data) != 0)  return 1;

	// Wait fpr stop condition
	if (i2cStop() != 0) return 1;

	// TIMING !!! Storing after i2c_stop! takes up to 10ms !!!
	i2c_delay(20000);
	// return 0: succesful write op
    return 0;
}
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
void i2c_erase_eeprom(void) {
  VMC_UINT_16 address;

  for (address = 0; address < 0xFFFF; address++) {
    i2c_storeByte(255, address);
	printf(".");
	}
}

//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
void i2c_read_eeprom(void) {
  VMC_UINT_16 address = 0;
  printf("-----------------------|\n");
  printf("address :: data |\n");

  for (address = 0; address < 0xFFFF; address++) {
    printf("%d :: %d |\n", address, i2c_loadByte(address) );
	i2c_delay(100000);
  }
  printf("-----------------------|\n");
}

//////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////////////////////////////////////////////
/// \brief Load a byte from the eeprom via i2c
/// \param Target Adress of the read operation
//////////////////////////////////////////////////////////////////////////////
VMC_UCHAR_8 i2c_loadByte(VMC_UINT_16 address) {

	// local char for data
    VMC_UCHAR_8 data = 0;

	// local char for address LSB
    VMC_UCHAR_8 addrlow = address;

	// local char for address MSB
    VMC_UCHAR_8 addrhigh = address >> 8;

	// send start condition to i2c
	i2c_start();

	// send device select code to i2c
	if (i2c_sendByte(0xa0) != 0) return 10;
	i2c_delay(100);

	// send address MSB
	if (i2c_sendByte(addrhigh) != 0) return 11;
	i2c_delay(100);

	// send address LSB
	if (i2c_sendByte(addrlow) != 0) return 12;
	i2c_delay(100);

	// repeat start condition
    i2c_start();

	// send second device select code for reading
	if (i2c_sendByte(0xa1) != 0) return 13;
	i2c_delay(100);

	// receive data from i2c
	data = i2c_getByte(data);

	// stop i2c communictaion
	if (i2cStop() == 1) return 14;

	// return readed data
	return data;
}
//////////////////////////////////////////////////////////////////////////////
//@}