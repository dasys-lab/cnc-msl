/*
 * $Id: FilterYUVExtractSubImages.cpp 2142 2007-04-15 10:49:00Z jewollen $
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
#include "FilterYUVExtractSubImages.h"


#include "../helpers/KeyHelper.h"


//#include "floatfann.h"
using namespace std;
using namespace castor;

FilterYUVExtractSubImages::FilterYUVExtractSubImages(int width, int height, int area_):Filter(OF_ZERO, width, height){

	printf("before malloc!!!\n");
    SystemConfigPtr sc = SystemConfig::getInstance();
        Configuration *vision = (*sc)["Vision"];

    UVYMAX = vision->tryGet<int>(80, "Vision", "UVYMAX", NULL);
    UVYMIN = vision->tryGet<int>(0, "Vision", "UVYMIN", NULL);


	area = area_;

	gray_image = (unsigned char *) malloc(area*area);
	uv_image = (unsigned char *) malloc(area*area);
	roi_image = (unsigned char *) malloc(area*area);
	roi_image_Roland = (unsigned char *) malloc(area*area);
	shadowlessGrey = (unsigned char *) malloc(area*area);

	if(gray_image == NULL)
		printf("gray_image = null!!!\n");

	if(uv_image == NULL)
		printf("uv_image = null!!!\n");

	if(uv_image == NULL)
		printf("roi_image = null!!!\n");

	lookupTable = (unsigned char *) malloc(256*256);
	lookupTableROI = (unsigned char *) malloc(256*256);
	lookupTableROIRoland = (unsigned char *) malloc(256*256);
	lookupTableUVY = (unsigned char *) malloc(256*256*256);
    lookupTableGreen = (unsigned char *) malloc(256*256);
  //      if(lookupTableUVY == NULL)
    //            printf("lookupTableUVY = null!!!\n");

	for(int y=0; y<256; y++) {
		for(int u=0; u<256; u++) {
			for(int v=0; v<256; v++) {
				if((int)v-(int)u - ((int)y/2)>0 && (int)v-(int)u - ((int)y/2)<255) lookupTableUVY[y*256*256 + u*256 + v] = (int)v-(int)u - ((int)y/2);
				else lookupTableUVY[y*256*256 + u*256 + v]=0;
			}
		}
	}
	

	if(lookupTable == NULL)
		printf("lookupTable = null!!!\n");

	if(lookupTableROI == NULL)
		printf("lookupTableROI = null!!!\n");

	
	init();
	initROI();
	initRoland();

	/*ifstream ifs("lookup.lot");
	double t;
	for(int i=0; i<256*256; i++) {
		ifs >> t;
		lookupTableROI[i] = (char)t;
	}*/

    loadAllLookupTables();

	/*ofstream ofs("lookupVision.lot");
	for(int i=0; i<256*256; i++) {
		ofs << (int)lookupTableROI[i] << "\t";
	}*/


    /*string filename = string(sc->getConfigPath())+string("/LookupTable");
    ifstream ifs(filename.c_str());
    if(ifs.is_open()) {
    double t;
        for(int u=0; u<256; u++)
        {
            for( int v=255; v>=0; v--)
            {
                ifs >> t;
                lookupTableGreen[u*256+v] = (unsigned char) t;
            }
        }
    }*/
}

void FilterYUVExtractSubImages::loadAllLookupTables() {
    SystemConfigPtr sc = SystemConfig::getInstance();

    loadLookupTable(string(sc->getConfigPath())+string("/")+sc->getHostname()+string("/LookupTableGreen.txt"), lookupTableGreen);
    loadLookupTable(string(sc->getConfigPath())+string("/")+sc->getHostname()+string("/LookupTable.txt"), lookupTable);
    loadLookupTable(string(sc->getConfigPath())+string("/")+sc->getHostname()+string("/LookupTable.txt"), lookupTableROI);
}

void FilterYUVExtractSubImages::loadLookupTable(string filename, unsigned char* lookup) {
  	ifstream ifs(filename.c_str());

        if(ifs.is_open()) {
	    double t;
	    for(int u=0; u<256; u++)
	    {
		for( int v=255; v>=0; v--)
		{
		    ifs >> t;
		    lookup[u*256+v] = (unsigned char) t;
		}
	    }
	    ifs.close();
	} else {
		cout << "Couldn't find LookupTable " << filename << endl;
		ROS_ERROR(string(string("Couldn't find LookupTable "+filename)).c_str());
        //exit(1);
	}
       
}

void FilterYUVExtractSubImages::loadLookupTable(string filename, unsigned char* lookup, int &min, int &max) {
}


FilterYUVExtractSubImages::~FilterYUVExtractSubImages(){

	cleanup();

}


void FilterYUVExtractSubImages::setLookupTableValue(int index, int value) {
    lookupTable[index] = value;
}

void FilterYUVExtractSubImages::setLookupTableROIValue(int index, int value) {
    lookupTableROI[index] = value;
}

void FilterYUVExtractSubImages::setLookupTableROIRolandValue(int index, int value) {
    lookupTableROIRoland[index] = value;
}

void FilterYUVExtractSubImages::setLookupTableUVYValue(int index, int value) {
    lookupTableUVY[index] = value;
}

int FilterYUVExtractSubImages::getLookupTableValue(int index) {
    return lookupTable[index];
}

int FilterYUVExtractSubImages::getLookupTableROIValue(int index) {
    return lookupTableROI[index];
}
int FilterYUVExtractSubImages::getLookupTableROIRolandValue(int index) {
    return lookupTableROIRoland[index];
}
int FilterYUVExtractSubImages::getLookupTableUVYValue(int index) {
    return lookupTableUVY[index];
}


		

void FilterYUVExtractSubImages::process(unsigned char * src, unsigned int width, unsigned int height, unsigned int mx, unsigned int my, unsigned char*  & gray_image_, unsigned char* & uv_image_, unsigned char* & roi_image_, unsigned char* & roi_image_Roland_, unsigned char* &shadowlessGrey_) {
		

		printf("FilterYUVExtractSubImages - mx=%d my=%d area=%d\n", mx, my, area);

		unsigned char * gray_tgt = gray_image;
		unsigned char * uv_tgt = uv_image;
		unsigned char * roi_tgt = roi_image;
        unsigned char * shadowlessGrey_tgt = shadowlessGrey;

		int startIndexX = mx - area/2;
		int startIndexY = my - area/2;

		//XXX nicht schön, aber macht die Sache einfacher!

		if(startIndexY % 2 != 0)
			startIndexY++;


		int lineSize = area;
		int area2 = area;

		register unsigned char u = 0;
		register unsigned char y = 0;
		register unsigned char v = 0;
		int value=0;
		register int color;

		if(KeyHelper::checkKey('a')) {
			UVYMAX+=2;
			printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
			SpicaHelper::vi->params.push_back(UVYMIN);
			SpicaHelper::vi->params.push_back(UVYMAX);
		}
		if(KeyHelper::checkKey('A')) {
			UVYMAX-=2;
			printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
                        SpicaHelper::vi->params.push_back(UVYMIN);
                        SpicaHelper::vi->params.push_back(UVYMAX);
		}
		if(KeyHelper::checkKey('q')) {
			UVYMIN+=2;
			printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
                        SpicaHelper::vi->params.push_back(UVYMIN);
                        SpicaHelper::vi->params.push_back(UVYMAX);
		}
		if(KeyHelper::checkKey('Q')) {
			UVYMIN-=2;
			printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
                        SpicaHelper::vi->params.push_back(UVYMIN);
                        SpicaHelper::vi->params.push_back(UVYMAX);
		}


		for(int i = 0; i < area2; i++){

			unsigned char * ptr = &(src[((startIndexX + i)*width + startIndexY)*2]);
			for(int j = 0; j < lineSize; j++){
				if(j%2==0)	
					u = *ptr++;
				else
					v = *ptr++;
				y = *ptr++;
				color = u*256 + v;
				//if(j==lineSize/2 && i==(area2/2)-100) {
				//	printf("Greencolor u: %d v: %d", (int)u, (int)v);
				//}

//				printf("i = %d j = %d\n", i, j);
			        value = (int)v-(int)u - ((int)y/2);
				//value = lookupTableUVY[y*256*256 + u*256 + v];
				if(value<UVYMAX && value > UVYMIN) value =255;
				else value=0;

				if(y <= 220 && lookupTable[color]>1)
					//*uv_tgt++ = lookupTable[color];
					*uv_tgt++ = 255;
					//*uv_tgt++ = value; //lookupTableROI[color];//lookupTable[color];
				else
					*uv_tgt++ = 0;

				*gray_tgt++ = y;


/*
				//if(*ptr <= 255)
					*uv_tgt++ = *(uv_tgt-1);//lookupTable[color];
				//else
				//	*uv_tgt++ = 0;
				*gray_tgt++ = *ptr++;
*/




				if(y <=220 && y>20 && lookupTable[color]>1)
					//*roi_tgt++ = lookupTable[color];
					*roi_tgt++ = 255;
					//*roi_tgt++ = value;//lookupTableROI[color];
					//*roi_tgt++ = lookupTableROIRoland[color];
				else
					*roi_tgt++ = 0;


/*

				//if(*ptr <= 255)
					*roi_tgt++ = *(roi_tgt-1);//lookupTableROI[color];
				//else
				//	*roi_tgt++ = 0;


*/
				//if(u<128 && v<128 && y>40 && y>215) *shadowlessGrey_tgt++ = y;
				//else *shadowlessGrey_tgt++ = y;
#ifdef OLDBALL
				if(y <= 180)
					*roi_Roland_tgt++ = lookupTableROI[color];//lookupTableROIRoland[color];
				else
					*roi_Roland_tgt++ = 0;
#endif

			}
		}

		gray_image_ = gray_image;
		uv_image_ = uv_image;
		roi_image_ = roi_image;
		roi_image_Roland_ = roi_image_Roland;
		shadowlessGrey_ = shadowlessGrey;
}



void FilterYUVExtractSubImages::process(unsigned char * src, unsigned int width, unsigned int height, unsigned int mx, unsigned int my, unsigned char*  & gray_image_, unsigned char* & uv_image_, unsigned char* & roi_image_)
{
        printf("FilterYUVExtractSubImages - mx=%d my=%d area=%d\n", mx, my, area);

        unsigned char * gray_tgt = gray_image;
        unsigned char * uv_tgt = uv_image;
        unsigned char * roi_tgt = roi_image;

        int startIndexX = mx - area/2;
        int startIndexY = my - area/2;

        //XXX nicht schön, aber macht die Sache einfacher!

        if(startIndexY % 2 != 0)
            startIndexY++;


        int lineSize = area;
        int area2 = area;

        register unsigned char u = 0;
        register unsigned char y = 0;
        register unsigned char v = 0;
        register int color;

        if(KeyHelper::checkKey('a')) {
            UVYMAX+=2;
            printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
            SpicaHelper::vi->params.push_back(UVYMIN);
            SpicaHelper::vi->params.push_back(UVYMAX);
        }
        if(KeyHelper::checkKey('A')) {
            UVYMAX-=2;
            printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
                        SpicaHelper::vi->params.push_back(UVYMIN);
                        SpicaHelper::vi->params.push_back(UVYMAX);
        }
        if(KeyHelper::checkKey('q')) {
            UVYMIN+=2;
            printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
                        SpicaHelper::vi->params.push_back(UVYMIN);
                        SpicaHelper::vi->params.push_back(UVYMAX);
        }
        if(KeyHelper::checkKey('Q')) {
            UVYMIN-=2;
            printf("Parameter Adjusted UVYMIN %d UVYMAX %d\n", UVYMIN, UVYMAX);
                        SpicaHelper::vi->params.push_back(UVYMIN);
                        SpicaHelper::vi->params.push_back(UVYMAX);
        }


        for(int i = 0; i < area2; i++){

            unsigned char * ptr = &(src[((startIndexX + i)*width + startIndexY)*2]);
            for(int j = 0; j < lineSize; j++)
            {
                /* Get uv */
                if(j%2==0)
                    u = *ptr++;
                else
                    v = *ptr++;

                /* Get gray */
                y = *ptr++;

                /* Calculate color */
                color = u*256 + v;

                /* Store uv */
                if(y <= 220)
                    *uv_tgt++ = lookupTable[color];
                else
                    *uv_tgt++ = 0;

                /* Store gray */
                *gray_tgt++ = y;

                /* Store roi */
                if( y>20 && y<=220 )
                    *roi_tgt++ = lookupTable[color];
                else
                    *roi_tgt++ = 0;
            }
        }

        gray_image_ = gray_image;
        uv_image_ = uv_image;
        roi_image_ = roi_image;
}


// void FilterYUVExtractSubImages::init(){
// 
// 
// 	int center = 128;
// 	double angle = atan2(220 - center, 50 - center); 
// 
// 	for(int u = 0; u < 256; u++){
// 		for(int v = 0; v < 256; v ++){  
// 
// 
// 			/*int value = (int) lrint(cos(angle)*(u-center) + sin(angle)*(v-center)) + 128;
// 			double diffAngle = fabs(atan2(v-center, u-center) - angle);
// 			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
// 			if(diffAngle > M_PI)
// 				diffAngle = fabs(diffAngle - 2.0*M_PI);
// 			value -= (int) lrint(diffAngle*180.0/M_PI*(0.5*distance + 0.5));
// 
// 			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
// 			//value = v;
// 			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);
// 			if(value < 0)
// 				value = 0;
// 			if(value > 255)
// 				value = 255;
// 
// 			lookupTable[u*256 + v] = (unsigned char) value; */
// 
// 			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) + 128)*2.0);
// 			double diffAngle = fabs(atan2(v-center, u-center) - angle);
// 			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
// 			if(diffAngle > M_PI)
// 				diffAngle = fabs(diffAngle - 2.0*M_PI);
// 			value -= (int) lrint(5.0*diffAngle*180.0/M_PI*(0.5*distance + 0.5));
// 
// 			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
// 			//value = v;
// 			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);
// 
// 
// 			//if(u > 110)
// 			//	value = 0;
// 
// 			if(value < 0)
// 				value = 0;
// 			if(value > 255)
// 				value = 255;
// 
// 			lookupTable[u*256 + v] = (unsigned char) value; 
// 
// 
// 
// 
// 
// 		}
// 	}
// 
// 
// }


void FilterYUVExtractSubImages::init(){
	int center = 128;
	//220 - 50
	//Orange Ball:
	//double angle = atan2(220 - center, 50 - center); 
	//Yellow Ball:
	double angle = atan2(150 - center, 70 - center);


// 	double minO=10;
// 	double maxO=-10;
// 
// 	fann_type input[2];
// 	fann_type *calc_out;
// 
// 	struct fann *ann = fann_create_from_file("fann/COI.net");
// 	for(int u = 0; u < 256; u++){
// 		for(int v = 0; v < 256; v ++){
// 			input[0] = v;
//     			input[1] = u;
// 
// 			calc_out = fann_run(ann, input);
// 
// 			double val = calc_out[0];
// 			if(val>maxO) maxO = val;
// 			if(val<minO) minO = val;
// 		}
// 	}

	for(int u = 0; u < 256; u++){
		for(int v = 0; v < 256; v ++){  


			/*int value = (int) lrint(cos(angle)*(u-center) + sin(angle)*(v-center)) + 128;
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value -= (int) lrint(diffAngle*180.0/M_PI*(0.5*distance + 0.5));

			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
			//value = v;
			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTable[u*256 + v] = (unsigned char) value; */

			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) + 128)*2.5);
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value -= (int) lrint(7.5*diffAngle*180.0/M_PI*(0.3*distance + 1.0));
			//value = 0;
			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
			int value2 = 0;//(int) lrint( (((double)(pow(v, 1.8)-u))/64.0)-0.1*diffAngle*(180.0/M_PI) );
			//value = ((int) lrint( (((double)(pow(v, 1.88)-u))/64.0)-0.1*diffAngle*(180.0/M_PI) ));
			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);

			//sehr cool, weil geht!! GO09
			//value = distance*255.0*2.07;

			if(value2 > value)
				value = value2;

			//value = v;

			//value = (int) lrint(((v - u) + 255.0)/2.0);

			//if(u > 110)
			//	value = 0;

			/* NN
			input[0] = v;
    			input[1] = u;

			calc_out = fann_run(ann, input);

			value = (int) ((minO+calc_out[0])*(256.0/(maxO-minO)));
			*/
//Witschs lines are from Endy?	
			//value = 0;
//


			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTable[u*256 + v] = (unsigned char) value; 
		}
	}


}

void FilterYUVExtractSubImages::initROI(){


// 	double minO=10;
// 	double maxO=-10;
// 
// 	fann_type input[2];
// 	fann_type *calc_out;
// 
// 	struct fann *ann = fann_create_from_file("fann/COI.net");
// 	for(int u = 0; u < 256; u++){
// 		for(int v = 0; v < 256; v ++){
// 			input[0] = v;
//     			input[1] = u;
// 
// 			calc_out = fann_run(ann, input);
// 
// 			double val = calc_out[0];
// 			if(val>maxO) maxO = val;
// 			if(val<minO) minO = val;
// 		}
// 	}

	int center = 128;
	//Orange Ball:
	double angle = atan2(220 - center, 50 - center); 
	//Yellow Ball:	
	//double angle = atan2(150 - center, 70 - center);

	for(int u = 0; u < 256; u++){
		for(int v = 0; v < 256; v ++){  



			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) + 128)*2.0);
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value -= (int) lrint(7.0*diffAngle*180.0/M_PI*(0.5*distance + 0.5));

			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
			//value = v;
			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);

			//value = (int) lrint(((v*v - u*u) + 65025)/510.0);

			//value = (int)lrint((pow(v,1.81) - u)/64.0);
			//value -= (int) lrint(0.15*diffAngle*180.0/M_PI);

			//value += (int) lrint((1.0 - distance)*50.0);
//*(0.5*distance + 0.5));

			//if(u > 110)
			//	value = 0;

			//damit haben wir GO09 gespielt
			//value = (int)lrint((pow(v,2.0) - u)/64.0);
			//value -= (int) lrint(4.0*diffAngle*180.0/M_PI);
			//value -= (int) lrint(2.5*diffAngle*180.0/M_PI);

			
// 			input[0] = v;
//     			input[1] = u;
// 
// 			calc_out = fann_run(ann, input);
// 
// 			value = (int) ((minO+calc_out[0])*(256.0/(maxO-minO)));
			
			//110
			if(u > 120) value=0;
			if(distance<30.0/128.0)
				value = 0;
//Endy lines
			//value = 0;
//
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTableROI[u*256 + v] = (unsigned char) value; 





		}
	}


}



void FilterYUVExtractSubImages::initRoland() {


	int center = 128;
	double angle = atan2(220 - center, 50 - center); 

	for(int u = 0; u < 256; u++){
		for(int v = 0; v < 256; v ++){  


			/*int value = (int) lrint(cos(angle)*(u-center) + sin(angle)*(v-center)) + 128;
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value -= (int) lrint(diffAngle*180.0/M_PI*(0.5*distance + 0.5));

			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
			//value = v;
			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);
			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTable[u*256 + v] = (unsigned char) value; */

			int value = (int) lrint(((cos(angle)*(u-center) + sin(angle)*(v-center)) + 128)*2.0);
			double diffAngle = fabs(atan2(v-center, u-center) - angle);
			double distance = sqrt((v-center)*(v-center) + (u-center)*(u-center))/128.0;
			if(diffAngle > M_PI)
				diffAngle = fabs(diffAngle - 2.0*M_PI);
			value -= (int) lrint(10.0*diffAngle*180.0/M_PI*(1.5*distance + 0.5));

			//int value = v; //(int) lrint(sqrt(2.0)*((v - 128) - (u - 128)) + 128);
			//value = v;
			//value = (int) lrint((1.0/sqrt(2.0))*((v - 128) - (u - 128)) + 128);


		//	if(u > 110)
		//		value = 0;

			if(value < 0)
				value = 0;
			if(value > 255)
				value = 255;

			lookupTableROIRoland[u*256 + v] = (unsigned char) value; 


		}
	}


}




void FilterYUVExtractSubImages::cleanup(){
	
	if(gray_image != NULL)	free(gray_image);
	if(uv_image != NULL)	free(uv_image);
	if(roi_image != NULL)	free(roi_image);
	if(lookupTable != NULL)	free(lookupTable);

}

