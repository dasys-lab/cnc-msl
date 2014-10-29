/*
 * $Id: FilterSobelGradient.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef FilterAdjustGainDirected_H
#define FilterAdjustGainDirected_H

#include "Filter.h"
#include "../CameraQuickCam.h"
//#include "../Camera1394.h"


class FilterAdjustGainDirected : public Filter {


	public:
		FilterAdjustGainDirected(int _width, int _height, int g_gain, int g_exposure);
		~FilterAdjustGainDirected();
		
		int process(CameraQuickCam *camera, unsigned char * src, int avgInt, int iwidth, int iheight);
		//int process(Camera1394 *camera, unsigned char * src, int avgInt, int iwidth, int iheight);


	protected:
		int gain;
		int exposure; 
		void init();
		void cleanup();

};




#endif

