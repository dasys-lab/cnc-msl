/*
 * $Id: XVDisplay.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef XVDisplay_H
#define XVDisplay_H


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <unistd.h>
#include <getopt.h>


#include <X11/Xlib.h>
#include <X11/Xutil.h>
#include <X11/extensions/Xvlib.h>
#include <X11/keysym.h>

#include <dc1394/conversions.h>

#ifndef XV_YV12
#define XV_YV12 0x32315659
#endif

#ifndef XV_YUY2
#define XV_YUY2 0x32595559
#endif

#ifndef XV_UYVY
#define XV_UYVY 0x59565955
#endif

class XVDisplay
{
	
	public:
		XVDisplay(long width, long height, long _format);
		~XVDisplay();
		

		static void iyu12yuy2 (unsigned char *src, unsigned char *dest, int NumPixels);
		static void rgb2yuy2 (unsigned char *RGB, unsigned char *YUV, int NumPixels);

		void displayFrameYUV(char * cameraBuffer);
		void displayFrameYUV(unsigned char * cameraBuffer);
		void displayFrameRGB(char * cameraBuffer);
		void displayFrameRGB(unsigned char * cameraBuffer);
		void displayFrameGRAY(char * cameraBuffer);
		void displayFrameGRAY(unsigned char * cameraBuffer);
		void setTitle(char * title);

	protected:

		void cleanup();
		void QueryXv();

		Display *display;
		Window window;
		long device_width;
		long device_height;
		int connection;
		XvImage *xv_image;
		XvAdaptorInfo *info;
		long format;
		GC gc;

		int adaptor;

		unsigned char * frame_buffer;
		unsigned char * tmp_buffer;



};


inline
void XVDisplay::iyu12yuy2 (unsigned char *src, unsigned char *dest, int NumPixels) {
  int i=0,j=0;
  register int y0, y1, y2, y3, u, v;
  while (i < NumPixels*3/2)
    {
      u = src[i++];
      y0 = src[i++];
      y1 = src[i++];
      v = src[i++];
      y2 = src[i++];
      y3 = src[i++];

      dest[j++] = y0;
      dest[j++] = u;
      dest[j++] = y1;
      dest[j++] = v;

      dest[j++] = y2;
      dest[j++] = u;
      dest[j++] = y3;
      dest[j++] = v;
    }
};

inline
void XVDisplay::rgb2yuy2 (unsigned char *RGB, unsigned char *YUV, int NumPixels) {
  int i, j;
  register int y0, y1, u0, u1, v0, v1 ;
  register int r, g, b;

  for (i = 0, j = 0; i < 3 * NumPixels; i += 6, j += 4)
    {
      r = RGB[i + 0];
      g = RGB[i + 1];
      b = RGB[i + 2];
      RGB2YUV (r, g, b, y0, u0 , v0);
      r = RGB[i + 3];
      g = RGB[i + 4];
      b = RGB[i + 5];
      RGB2YUV (r, g, b, y1, u1 , v1);
      YUV[j + 0] = y0;
      YUV[j + 1] = (u0+u1)/2;
      YUV[j + 2] = y1;
      YUV[j + 3] = (v0+v1)/2;
    }
}




#endif

