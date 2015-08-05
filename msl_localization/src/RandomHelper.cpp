/*
 * $Id: RandomHelper.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "RandomHelper.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/time.h>

RandomGaussHelper::RandomGaussHelper(){
	LookupTable = NULL;

	init();
	initRandom();
}


RandomGaussHelper::~RandomGaussHelper(){
	cleanup();
}


void RandomGaussHelper::init(){


	LookupTable = (double *) malloc(((RAND_MAX >> 16) + 1)*sizeof(double));

	double resolution = 0.0005;

	int tmpLookupSize = 8*(int)(1/resolution)+1;
	
	double * tmpLookup = (double *) malloc(tmpLookupSize*sizeof(double));
	double * tmpLookupReal = (double *) malloc(tmpLookupSize*sizeof(double));

	double x = -4.0;
	tmpLookup[0] = 1.0/sqrt(2.0*M_PI)*exp(-(x*x)/2.0) * resolution;	
	tmpLookupReal[0] = x;	

	for(int i = 1; i < tmpLookupSize; i++){

		x += resolution;
		tmpLookup[i] = (1.0/sqrt(2.0*M_PI)*exp(-(x*x)/2.0)*resolution) + tmpLookup[i-1];
		tmpLookupReal[i] = x;
	

	}

	

	int beg = 0;

	for(int i = 0; i <= (RAND_MAX >> 16) ; i++){

		double value = i*1.0/(RAND_MAX >> 16);
		while(beg < tmpLookupSize && tmpLookup[beg] < value){
			beg++;
		}
		if(beg >= tmpLookupSize){
			beg = tmpLookupSize - 1;
		}
		LookupTable[i] = tmpLookupReal[beg];

	}

	free(tmpLookup);
	free(tmpLookupReal);
}

void RandomGaussHelper::cleanup(){

	if(LookupTable != NULL)
		free(LookupTable);



}

double RandomGaussHelper::getRandomGauss(){

	unsigned int r = rand();
	return (LookupTable[r >> 16]);

}

void RandomGaussHelper::initRandom()
{
   struct timeval t;



   gettimeofday(&t, NULL);
   srand(t.tv_usec);

}

void RandomHelper::initRandom(){


   struct timeval t;
   gettimeofday(&t, NULL);
   srand(t.tv_usec);


}

double RandomHelper::rand01(){
	return (rand()/((double)RAND_MAX));
}


