/*
 * spi.h
 *
 *  Created on: 21.01.2011
 *      Author: philipp
 */

#ifndef SPI_H_
#define SPI_H_

#include "defaults.h"
#include "global.h"

#define NCS_Low() RESET(P_CS)
#define NCS_High() SET(P_CS)

void spi_init(void);
uint8_t spi_exch (uint8_t output);

#endif /* SPI_H_ */
