#include <inttypes.h>

void wait(int16_t ms)
{
	uint16_t i, j;
	
	for (i = 0; i < ms; i++) {
		for (j = 0; j < 16000; j++) {
			asm volatile("nop\n\t"
					::);
		}
	}

}
