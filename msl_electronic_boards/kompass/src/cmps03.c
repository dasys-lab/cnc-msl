/* $Id: cmps03.c,v 1.2 2003/11/15 22:46:20 bsd Exp $ */

#include <compat/twi.h>

#include <inttypes.h>

#include "cmps03.h"
#include "i2c.h"
#include "cmpsHelper.h"
#include <util/delay.h>

/*
 * calibrate the cmps03
 */
int8_t cmps03_calibrate(void)
{
	// at first set 0xff to the register 0x0f of the device 0x60

	// start condition
	if(i2c_start(0x08, 1))return -1;

	// address slave device, write
	if(i2c_sla_rw(0x60, 0, TW_MT_SLA_ACK, 1))return -2;

	// write register address
	if(i2c_data_tx(0x0f, TW_MT_DATA_ACK, 1))return -3;

	// repeated start condition
  	if(i2c_start(0x10, 1))return -4;

   	// address slave device, write 
  	if (i2c_sla_rw(0x60, 0, TW_MR_SLA_ACK, 1))return -5;

  	// write data
  	if (i2c_data_tx(0xff, TW_MR_DATA_ACK, 1))return -6;
 
	// stop i2c
	if (i2c_stop())return -7;

	//wait a time
	_delay_us(2000000);
	
	// after calibrationg set 0x00 to the register 0x0f of the device 0x60 

	// start condition
	if(i2c_start(0x08, 1))return -1;

	//address slave device, write
	if(i2c_sla_rw(0x60, 0, TW_MT_SLA_ACK, 1))return -2;

	// write register address
	if(i2c_data_tx(0x0f, TW_MT_DATA_ACK, 1))return -3;

	// repeated start condition
  	if(i2c_start(0x10, 1))return -4;

   	// address slave device, write
  	if (i2c_sla_rw(0x60, 0, TW_MR_SLA_ACK, 1))return -5;

  	// write data
  	if (i2c_data_tx(0x00, TW_MR_DATA_ACK, 1))return -6;
	
	// stop i2c 
	if (i2c_stop())return -7;

	return 0;
}

/*
 * read the data byte at the specified address from the specified device
*/ 
int8_t cmps03_get_word(uint8_t device, uint8_t addr, uint16_t * value)
{
  uint8_t v1, v2;

  // start condition 
  if (i2c_start(0x08, 1))
    return -1;

  // address slave device, write
  if (i2c_sla_rw(device, 0, TW_MT_SLA_ACK, 1))
    return -2;

  // write register address
  if (i2c_data_tx(addr, TW_MT_DATA_ACK, 1))
    return -3;

  // repeated start condition
  if (i2c_start(0x10, 1))
    return -4;

   // address slave device, read
  if (i2c_sla_rw(device, 1, TW_MR_SLA_ACK, 1))
    return -5;

  // read data byte 1
  if (i2c_data_rx(&v1, I2C_ACK, TW_MR_DATA_ACK, 1))
    return -6;

  // read data byte 2
  if (i2c_data_rx(&v2, I2C_NACK, TW_MR_DATA_NACK, 1))
    return -7;

  if (i2c_stop())
    return -8;

  *value = (v1 << 8) | v2 ;

  return 0;
}


int16_t bearing16(void)
{
  uint16_t d;
  int8_t rc;

  rc = cmps03_get_word(0x60, 2, &d);
  if (rc < 0) {
    return -1;
  }

  return d;
}



