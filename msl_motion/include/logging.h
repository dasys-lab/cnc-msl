#ifndef logging_h
#define logging_h 1

#include <stdio.h>
#include <string>
#include "gonzales.h"
#include "driver/eposcan.h"

#include <SystemConfig.h>
#include <Configuration.h>


extern gonzales_state gonz_state;


void logging_init();
void logData();



#endif
