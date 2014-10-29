/*
 * $Id: FilterLinePoints.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef FilterLinePoints3D_H
#define FilterLinePoints3D_H

#include <string>

#include "../helpers/ScanLineHelper3D.hpp"
#include "../helpers/Distance3DHelper.hpp"


class FilterLinePoints3D : private ScanLineHelper3D, private Distance3DHelper
{
	public:
		FilterLinePoints3D();
		~FilterLinePoints3D();
		
		void process(unsigned char *&src, unsigned char * const &mask);
		void panno(unsigned char *&src, unsigned char * const &mask, ImageSize iSize, ImageSize oSize);
		
	protected:
		double FieldLength;
		double FieldWidth;
		double GoalAreaWidth;
		double GoalAreaLength;
		double MiddleCircleRadius;
		double GoalInnerAreaWidth;
		double GoalInnerAreaLength;
		double CornerCircleRadius;
		double LineWidth;
		double GoalWidth;
		bool GoalInnerAreaExists;
		bool CornerCircleExists;
		
		uint16_t width;
		uint16_t height;
		
		uint16_t centerX;
		uint16_t centerY;
		
		uint16_t * linesInner;
		uint32_t * linesInnerOffset;
		uint16_t * linesOuter;
		uint32_t * linesOuterOffset;
		uint16_t maxPoints;
		uint16_t nLines;
		
		uint8_t LinePointsThreshold;
		uint8_t FloorBrightness;
		uint8_t LinePointsJump;
		uint8_t MinInnerLineWidth;
		uint8_t MinOuterLineWidth;
		uint8_t MaxInnerLineWidth;
		uint8_t MaxOuterLineWidth;
		
		// For panno
		uint16_t iRadiusStart;
		uint16_t iRadiusEnd;
		uint16_t oRadiusStart;
		uint16_t oRadiusEnd;
		
		double * holders;
		
		bool * angleValidity;
		
		std::vector<double> LinePoints3Dx;
		std::vector<double> LinePoints3Dy;
		std::vector<double> LinePoints3Dz;
		
		FILE *plot;
		FILE *logFile;
		std::string confPath;
		std::string filePath;
};


#endif

