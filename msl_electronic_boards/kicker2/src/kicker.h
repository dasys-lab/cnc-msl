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

#ifndef KICKER_H__
#define KICKER_H__

typedef enum {
	POS_MIDDLE = 0,
	POS_LEFT   = 1,
	POS_RIGHT  = 2
} POSITION_SERVO;

// ==== prototypes (public) ====
void kicker_init(void);
void kicker_softreset(void);
uint8_t kicker_rotate(uint8_t);
uint8_t kicker_set_pulse_width(uint8_t, uint16_t);
uint16_t kicker_get_pulse_width(uint8_t);
uint8_t kicker_shoot(uint8_t);
uint8_t kicker_uext(uint16_t);
uint8_t kicker_ext_active(void);
uint8_t kicker_lext(uint16_t);
uint8_t kicker_rext(uint16_t);
uint8_t kicker_check_time_delta(uint32_t timestamp, uint16_t min_delta);

extern uint16_t ext_min_sleep;
extern uint16_t ext_max_out;
#endif
