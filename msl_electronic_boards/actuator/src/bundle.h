#ifndef BUNDLE_H__
#define BUNDLE_H__

#include "defaults.h"
#include <avr/io.h>
#include "global.h"
#include "mcp2515.h"
#include "messages.h"

//tExtendedCAN message;

void	bundle_init(void);
void	bundleSendStatus(uint32_t time);

#endif
