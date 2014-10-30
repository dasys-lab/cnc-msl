/*
 * $Id: FilterYUVToRGB.cpp 1531 2006-08-01 21:36:57Z phbaer $
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
#include "FilterYUVToRGB.h"

#include <algorithm>

FilterYUVToRGB::FilterYUVToRGB(int width, int height):Filter(OF_RGB, width, height) {

	init();

}



FilterYUVToRGB::~FilterYUVToRGB(){

	cleanup();

}
		

unsigned char * FilterYUVToRGB::process(unsigned char * src, unsigned int imagesize){
    
    unsigned char * tgt = outputBuffer;

    for (unsigned int i = 0; i < imagesize; i += 4) {
      register int u  = src[i];
      register int y0 = src[i+1];
      register int v  = src[i+2];
      register int y1 = src[i+3];

      *(tgt++) = t_r[(y0<<8)|v];
      *(tgt++) = t_g2[(y0<<8)|t_g1[(u<<8)|v]];
      *(tgt++) = t_b[(y0<<8)|u];

      *(tgt++) = t_r[(y1<<8)|v];
      *(tgt++) = t_g2[(y1<<8)|t_g1[(u<<8)|v]];
      *(tgt++) = t_b[(y1<<8)|u];
    }

    return outputBuffer;


}



void FilterYUVToRGB::init(){

    for (int i = 0; i < 256; i++) {
      for (int j = 0; j < 256; j++) {

        int r = i + (((j - 128) * 1434) / 2048);
        int b = i + (((j - 128) * 2078) / 2048);
        int g1 = (((i - 128) * 406) / 2048) + (((j - 128) * 595) / 2048);
        int g2 = i - j;

        t_r[(i << 8) | j] = std::min(std::max(0, r), 255);
        t_b[(i << 8) | j] = std::min(std::max(0, b), 255);
        t_g1[(i << 8) | j] = std::min(std::max(0, g1), 255);
        t_g2[(i << 8) | j] = std::min(std::max(0, g2), 255);
      }
    }

}


void FilterYUVToRGB::cleanup(){




}

