/*
 * srf08.h
 *
 *  Created on: 30.11.2010
 *      Author: philipp
 */

#ifndef SRF08_H_
#define SRF08_H_

#define START_MEASUREMENT_CM 0x51

#define AMP_COMMAND 0x01
#define RANGE_COMMAND 0x02
#define AUTOCAL_COMMAND 0x03

void initSRF(void);

unsigned char srf08_ready(void);

void setAmpValue(void);
void setSpecAmpValue(unsigned char value);

void setRangeValue(void);
void setSpecRangeValue(unsigned char value);

int getSensorRevision(void);

unsigned int getRange(void);

void autoCalibration(void);

#endif /* SRF08_H_ */
