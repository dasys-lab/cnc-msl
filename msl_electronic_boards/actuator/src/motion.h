#ifndef MOTION_H__
#define MOTION_H__

#include "defaults.h"
#include <avr/io.h>
#include "global.h"
#include "mcp2515.h"
#include "messages.h"

//tExtendedCAN message;

void	motion_init();
void	motionSendStatus(uint32_t time);

#endif
