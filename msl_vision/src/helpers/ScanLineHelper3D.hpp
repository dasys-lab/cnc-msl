/*
 * $Id: ScanLineHelper3D.h 1531 2006-08-01 21:36:57Z phbaer $
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
#ifndef ScanLineHelper3D_H
#define ScanLineHelper3D_H

#include <stdint.h>

class ScanLineHelper3D
{
	public:
		ScanLineHelper3D();
		~ScanLineHelper3D();
		
		/*!
		 * Setup the line Points
		 * Input:	startX, startY, endX, endY
		 * Output:	line, nPoints
		 * Bresenham - Algorithm
		 */
		uint16_t GetLinePoints(uint16_t * &line, double &startX, double &startY, double &endX, double &endY);
		
		/*!
		 * Returns the array of the lines in the inner area
		 */
		uint16_t * getLinesInner() const;
		uint16_t * getLinesOuter() const;
		
		uint32_t * getLinesInnerOffsets() const;
		uint32_t * getLinesOuterOffsets() const;
		
		uint16_t	getNumberLines() const;
		uint16_t	getMaxPoints() const;
		uint16_t	getInnerRadiusStart() const;
		uint16_t	getInnerRadiusEnd() const;
		uint16_t	getOuterRadiusStart() const;
		uint16_t	getOuterRadiusEnd() const;
		
		double	* getHolders() const;
		
	protected:		
		void init();
		double grad2rad(double grad);
		
		uint16_t iRadiusStart;
		uint16_t iRadiusEnd;
		uint16_t oRadiusStart;
		uint16_t oRadiusEnd;
		uint16_t iRadiusOffset;
		
		/*
		 * Structure of the Array
		 * FirstStart, FirstEnd;
		 * SecondStart, SecondEnd;
		 * ThirdStart, ThirdEnd;
		 */
		double * holders;
		
		uint16_t * linesInner;
		uint16_t * linesOuter;
		
		uint32_t * linesInnerOffsets;
		uint32_t * linesOuterOffsets;
		
		uint16_t nLines;
		uint16_t maxPoints;
		
		uint16_t width;
		uint16_t height;
		
		uint16_t centerX;
		uint16_t centerY;
};

#endif
