#ifndef VISION_H__
#define VISION_H__

#include "defaults.h"
#include <avr/io.h>
#include "global.h"
#include "mcp2515.h"
#include "messages.h"

//tExtendedCAN message;

void	vision_init();
void	visionSendStatus(uint32_t time);

#endif
