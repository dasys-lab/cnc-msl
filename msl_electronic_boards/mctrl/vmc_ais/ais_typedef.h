

#ifndef AIS_TYPEDEF_H
#define AIS_TYPEDEF_H

#define _FIRMWWARE_VER_ "VMC Firmware Version 0.95z"

#define _NUM_MOTORS_ 3
#define SIGNAL_RANGE 1023     // Signal range -1024 to +1024

#define PI 3.1415926535897932384626433832795


#define ABS(a)   ( ((a) > 0) ? (a) : -(a) )
#define MAX(a,b) ( ((a) > (b)) ? (a) : (b) )
#define MIN(a,b) ( ((a) < (b)) ? (a) : (b) )
#define _IS_MOTORID_(a) (((a)>=0) && (a) <= _NUM_MOTORS_ )		//@@@@@@@@@@@@@@@@@@@@@@@@@



typedef unsigned int VMC_UINT_16;
typedef int VMC_INT_16;
typedef unsigned long TMC_ULONG_32;
typedef long VMC_LONG_32;
typedef unsigned char VMC_UCHAR_8;
typedef char VMC_CHAR_8;
typedef signed char VMC_SCHAR_8;
typedef double VMC_DOUBLE;
typedef float VMC_FLOAT;

#endif
