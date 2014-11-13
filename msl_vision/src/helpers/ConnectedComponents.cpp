

#include "ConnectedComponents.h"

#define _TEST
#ifdef _TEST

#include <functional>
#include <stdlib.h>
#include <stdio.h>



void test_inline_img()
{
    const  char *_img = {
	"              "
	"    *  0  *   "
	"   **  0  *   "
	"    *******   "
	"      *       "
	"   *          "
	"  ***         "
    };
    const unsigned char *img = (const unsigned char *)_img;
    int width = 14, height = 7;

    unsigned char *out_uc = (unsigned char *)malloc(width*height);

    ConnectedComponents cc(30);
    cc.connected(img, out_uc, width, height,
		 std::equal_to<unsigned char>(),
		 false);

    for(int r=0; r<height; ++r) {
	for(int c=0; c<width; ++c)
	    putchar('0'+out_uc[r*width+c]);
	putchar('\n');
    }

    free(out_uc);
}



void ConnectedComponents::bwlabel(unsigned char *img, unsigned int *out, int width, int height)
{          

    ConnectedComponents cc(255);
    cc.connected(img, out, width, height,
		 std::equal_to<unsigned char>(),
		 constant<bool,true>());
}


main()
{
    test_inline_img();
    test_raw_img("img.raw", "out.raw", 321, 241);
}