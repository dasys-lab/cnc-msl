#ifndef BUTTON_H__
#define BUTTON_H__

#include "defaults.h"
#include <avr/io.h>
#include "global.h"
#include "mcp2515.h"
#include "messages.h"

//tExtendedCAN message;

void	button_init();
void	buttonSendStatus(uint32_t time);

#endif
