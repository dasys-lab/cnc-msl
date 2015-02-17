#ifndef __i2c_h__
#define __i2c_h__

#include <inttypes.h>

/*
 * define some handy I2C constants
 */
#define I2C_START     (_BV(TWINT)|_BV(TWSTA)|_BV(TWEN))
#define I2C_MASTER_TX (_BV(TWINT)|_BV(TWEN))
#define I2C_TIMEOUT   1000
#define I2C_ACK       1
#define I2C_NACK      0

int8_t i2c_stop(void);

void i2c_error(const char * message, uint8_t cr, uint8_t status);

int8_t i2c_start(uint8_t expected_status, uint8_t verbose);

int8_t i2c_sla_rw(uint8_t device, uint8_t op, uint8_t expected_status, 
                  uint8_t verbose);

int8_t i2c_data_tx(uint8_t data, uint8_t expected_status, uint8_t verbose);

int8_t i2c_data_rx(uint8_t * data, uint8_t ack, uint8_t expected_status, 
                   uint8_t verbose);

#endif
