/*
 * config.h
 
	The contents of this file were originally from the CAN Software Library
	provided by Atmel written for AT90CAN devices. Use of this file is subject
	to Atmel's End User License Agreement.
 
 */ 

//******************************************************************************
//! @file $RCSfile: config.h,v $
//!
//! Copyright (c) 2007 Atmel.
//!
//! Use of this program is subject to Atmel's End User License Agreement.
//! Please read file license.txt for copyright notice.
//!
//! @brief Configuration file for the following project:
//!             - can_sensor_node_example_gcc
//!
//! This file can be parsed by Doxygen for automatic documentation generation.
//! This file has been validated with AVRStudio-413528/WinAVR-20070122.
//!
//! @version $Revision: 3.20 $ $Name: jtellier $
//!
//! @todo
//! @bug
//******************************************************************************

#ifndef _CONFIG_H_
#define _CONFIG_H_

//_____ I N C L U D E S ________________________________________________________
#include <avr/io.h>
#include <avr/interrupt.h>

//_____ M A C R O S ____________________________________________________________
//! Some useful macros...
// Max(a, b): Take the max between a and b
// Min(a, b): Take the min between a and b
// Align_up(val, n):   Around (up)   the number (val) on the (n) boundary
// Align_down(val, n): Around (down) the number (val) on the (n) boundary
#define Max(a, b)          ( (a)>(b) ? (a) : (b) )
#define Min(a, b)          ( (a)<(b) ? (a) : (b) )

typedef uint8_t Bool;

//_____ D E F I N I T I O N S __________________________________________________

    // -------------- MCU LIB CONFIGURATION
#define FOSC           8000        // 8Mhz external clock
#define F_CPU          (FOSC*1000) // Need for AVR GCC

    // -------------- CAN LIB CONFIGURATION
#define CAN_BAUDRATE   250        // in kBit
//#define CAN_BAUDRATE   CAN_AUTOBAUD

    // -------------- MISCELLANEOUS

//_____ D E C L A R A T I O N S ________________________________________________

#endif  // _CONFIG_H_