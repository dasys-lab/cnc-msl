#ifndef LOOKUPTABLE_H
#define LOOKUPTABLE_H

#define COLOR_UDEF 0
#define COLOR_RED 1
#define COLOR_GREEN 2
#define COLOR_BLUE 3
#define COLOR_YELLOW 4
#define COLOR_BLACK 5
#define COLOR_COUNT 6
#define LOOKUPTABLE_SIZE 140
#define LOOKUPTABLE_OFFSET 60


typedef struct LuminanceLimit{
	unsigned char low;
	unsigned char high;
} LuminanceLimit;

typedef struct Lookuptable{
	unsigned char table[LOOKUPTABLE_SIZE][LOOKUPTABLE_SIZE];				
	LuminanceLimit limits[COLOR_COUNT];	
} Lookuptable;



#endif
