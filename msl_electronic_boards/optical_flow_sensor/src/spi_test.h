/*
 * spi.h
 *
 *  Created on: 14.01.2013
 *      Author: timo
 */

void InitSPI(void);
void WriteByteSPI(unsigned char byte);
char ReadByteSPI(char addr);