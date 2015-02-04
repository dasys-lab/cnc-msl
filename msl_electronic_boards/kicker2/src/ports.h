/*
 * $Id$
 *
 *
 * Copyright 2005,2006 Carpe Noctem, Distributed Systems Group,
 * University of Kassel. All right reserved.
 *
 * The code is derived from the software contributed to Carpe Noctem by
 * the Carpe Noctem Team.
 *
 * The code is licensed under the Carpe Noctem Userfriendly BSD-Based
 * License (CNUBBL). Redistribution and use in source and binary forms,
 * with or without modification, are permitted provided that the
 * conditions of the CNUBBL are met.
 *
 * You should have received a copy of the CNUBBL along with this
 * software. The license is also available on our website:
 * http://carpenoctem.das-lab.net/license.txt
 *
 *
 * <description>
 */

#ifndef PORTS_H__
#define PORTS_H__

#define ROTOR_SWITCH        PIND0

// globals
volatile static uint8_t keystate = 0; ///< debounced and inverted key state

// prototypes
void ports_init(void);
uint16_t ports_get_adc(uint8_t);

#endif
