/*
 * ballhandling.cpp
 *
 *  Created on: Mar 11, 2015
 *      Author: Lukas Will
 */



#include "ballhandling.h"

uint8 getError(BlackLib::BlackGPIO &ff1, BlackLib::BlackGPIO &ff2) {
	uint8 ff = (ff1.getValue() << 1) | ff2.getValue();

	switch (ff) {
		case	0:
			return none;		// Kein Fehler

		case	1:
			return bypass;		// Kurzschluss

		case	2:
			return temperature;	// Ueberhitzung

		case	3:
			return voltage;		// Unterspannung
	}
}

