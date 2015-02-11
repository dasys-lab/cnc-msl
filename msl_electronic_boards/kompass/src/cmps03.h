/* $Id: cmps03.h,v 1.2 2003/11/15 22:46:20 bsd Exp $ */

#ifndef __cmps03_h__
#define __cmps03_h__

#include <inttypes.h>

int8_t cmps03_get_byte(uint8_t device, uint8_t addr, uint8_t * value);

int8_t cmps03_get_word(uint8_t device, uint8_t addr, uint16_t * value);

int16_t bearing16(void);

uint8_t bearing8(void);

#endif
