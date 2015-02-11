/*
 * spi.h
 *
 *  Created on: 21.01.2011
 *      Author: philipp
 */

#ifndef SPI_H_
#define SPI_H_

#define NSS_Low() PORTA &= ~(1<<PA6)
#define NSS_High() PORTA |= (1<<PA6)

void SPI_Init(void);
unsigned int SPI_EXCH (unsigned char output);

#endif /* SPI_H_ */
