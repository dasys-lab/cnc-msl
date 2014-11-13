/*
 * $Id: XVDisplay.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "XVDisplay.h"


XVDisplay::XVDisplay(long width, long height, long _format):
	display(NULL),
	window((Window)NULL),
	connection(-1),
	xv_image(NULL),
	format(_format),
	adaptor(-1)
{



	XGCValues xgcv;
	long background=0x010203;

	display=XOpenDisplay(getenv("DISPLAY"));
	if(display==NULL)
	{
		fprintf(stderr,"Could not open display \"%s\"\n",getenv("DISPLAY"));
		cleanup();
		exit(-1);
	}

	QueryXv();

	if ( adaptor < 0 )
	{
		cleanup();
		exit(-1);
	}

	device_width = width;
	device_height = height;

	window=XCreateSimpleWindow(display,DefaultRootWindow(display),0,0,width,height,0,
		WhitePixel(display,DefaultScreen(display)),
		background);

	XSelectInput(display,window,StructureNotifyMask|KeyPressMask);
	XMapWindow(display,window);
	connection=ConnectionNumber(display);

	gc=XCreateGC(display,window,0,&xgcv);

	frame_buffer= (unsigned char *) malloc(width*height*3);
	tmp_buffer= (unsigned char *) malloc(width*height*3);

}


XVDisplay::~XVDisplay(){

	cleanup();

}

void XVDisplay::displayFrameYUV(char * cameraBuffer)
{
	format=XV_UYVY;

	if(adaptor>=0){

		memcpy(frame_buffer, cameraBuffer, device_width*device_height*2);

		xv_image=XvCreateImage(display,info[adaptor].base_id,format,(char *)frame_buffer,
			device_width,device_height);
		XvPutImage(display,info[adaptor].base_id,window,gc,xv_image,
			0,0,device_width,device_height,
			0,0,device_width,device_height);

		xv_image=NULL;
	}
}

void XVDisplay::displayFrameRGB(char * cameraBuffer)
{
	format=XV_YUY2;

	if(adaptor>=0){

		rgb2yuy2( (unsigned char *) cameraBuffer, frame_buffer, device_width*device_height);

		xv_image=XvCreateImage(display,info[adaptor].base_id,format,(char *)frame_buffer,
			device_width,device_height);
		XvPutImage(display,info[adaptor].base_id,window,gc,xv_image,
			0,0,device_width,device_height,
			0,0,device_width,device_height);

		xv_image=NULL;
	}
}


void XVDisplay::displayFrameGRAY(char * cameraBuffer)
{
	format=XV_YUY2;

	if(adaptor>=0){

		unsigned char * tgt_ptr = tmp_buffer;
		unsigned char * src_ptr = (unsigned char *) cameraBuffer;

		for(int i = 0; i < device_width*device_height; i++){
			*tgt_ptr++ = *src_ptr;
			*tgt_ptr++ = *src_ptr;
			*tgt_ptr++ = *src_ptr++;
		}

		rgb2yuy2( tmp_buffer, frame_buffer, device_width*device_height);

		xv_image=XvCreateImage(display,info[adaptor].base_id,format,(char *)frame_buffer,
			device_width,device_height);
		XvPutImage(display,info[adaptor].base_id,window,gc,xv_image,
			0,0,device_width,device_height,
			0,0,device_width,device_height);

		xv_image=NULL;
	}
}



void XVDisplay::cleanup(){

	if ((void *)window != NULL)
		XUnmapWindow(display,window);
	if (display != NULL)
		XFlush(display);
	if (frame_buffer != NULL)
		free( frame_buffer );
	if (tmp_buffer != NULL)
		free( tmp_buffer );


}

void XVDisplay::setTitle(char * title)
{
	XStoreName(display, window, title);
}

void XVDisplay::QueryXv()
{
	int num_adaptors;
	int num_formats;
	XvImageFormatValues *formats=NULL;
	int i,j;
	char xv_name[5];

	XvQueryAdaptors(display,DefaultRootWindow(display),(unsigned int *) &num_adaptors,&info);

	printf("Number of Adapters: %d\n", num_adaptors);

	for(i=0;i<num_adaptors;i++) {
		formats=XvListImageFormats(display,info[i].base_id,&num_formats);
		for(j=0;j<num_formats;j++) {
			xv_name[4]=0;
			memcpy(xv_name,&formats[j].id,4);
			if(formats[j].id==format) {
				fprintf(stderr,"using Xv format 0x%x %s %s\n",formats[j].id,xv_name,(formats[j].format==XvPacked)?"packed":"planar");
				if(adaptor<0)adaptor=i;
			}
		}
	}
		XFree(formats);
	if(adaptor<0)
		fprintf(stderr,"No suitable Xv adaptor found");

}
