#ifndef CN_FORCELOOKUP
#define CN_FORCELOOKUP
#include "main.h"
#include "cn_controller.h"


//force range should be max accel * 2 / controller frequency
#define FORCE_RANGE 100
#define LINE_WIDTH (FORCE_RANGE*2l+1)



//#define PI 3.1415926535897932
//#define TWO_PI (2*PI)
#define TWO_PI_THIRD (TWO_PI/3.0)
#define FOUR_PI_THIRD (2.0*TWO_PI/3.0)
#define PI_THIRD (PI/3.0)



typedef struct cn_map_point {
  
	sword x;
	sword y;
	
} map_point;

#define DMOD(x,y)	( (x) - floor((x)/(y)) * (y) )


void cn_init_force_lookup();
void cn_calc_force_lookup(float minAccel, float maxAccel);

#endif