#ifndef LIGHT_H__
#define LIGHT_H__

#include "defaults.h"
#include <avr/io.h>
#include "global.h"
#include "mcp2515.h"
#include "messages.h"

//tExtendedCAN message;

void	lightbarrier_init(void);
void	sendStatus(uint32_t time);

#endif
