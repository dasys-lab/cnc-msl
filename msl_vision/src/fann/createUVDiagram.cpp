#include <stdio.h>
#include "floatfann.h"
#include <fstream>

using namespace std;

int main()
{
    fann_type *calc_out;
    fann_type input[2];

    struct fann *ann = fann_create_from_file("COI.net");

    ofstream file("result.csv");
    
    calc_out = fann_run(ann, input);

    //printf("xor test (%f,%f) -> %f\n", input[0], input[1], calc_out[0]);

    for(int x=1; x<256; x++) {
         for(int y=1; y<256; y++) {
		input[0] = x;
    		input[1] = y;
		calc_out = fann_run(ann, input);
		file << calc_out[0] << "; ";
         }
	 file << endl;
    }

    fann_destroy(ann);
    return 0;
}
