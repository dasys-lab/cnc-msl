
#include "ScanLineHelper3D.hpp"

#include <SystemConfig.h>
#include <stdio.h>

using std::cout;
using std::endl;

extern uint16_t dir;

/*
** ScanLineHelper3D constructor
** Load all variables and start the initialisation
*/
ScanLineHelper3D::ScanLineHelper3D()
{
	cout << "Start ScanLineHelper3D Constructor" << endl;

	supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
	supplementary::Configuration *vision3D = (*sc)["Vision3D"];

	// Get configs
	cout << "ScanLines" << endl;
	cout << "\tInnerRadiusStart_";
	iRadiusStart	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusStart", NULL);
	cout << "\tInnerRadiusEnd_";
	iRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "InnerRadiusEnd", NULL);
	cout << "\tOuterRadiusStart_";
	oRadiusStart	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusStart", NULL);
	cout << "\tOuterRadiusEnd_";
	oRadiusEnd	= vision3D->get<uint16_t>("ScanLines", "OuterRadiusEnd", NULL);
	cout << "\tNumber_";
	nLines		= vision3D->get<uint16_t>("ScanLines", "Number", NULL);
	cout << "\tMaxPoints_";
	maxPoints	= vision3D->get<uint16_t>("ScanLines", "maxPoints", NULL);

	cout << "Image" << endl;
	cout << "\tWidth_";
	width	= vision3D->get<uint16_t>("Image", "Width", NULL);
	cout << "\tHeight_";
	height	= vision3D->get<uint16_t>("Image", "Height", NULL);
	cout << "\tOffsetX_";
	int16_t offsetX = vision3D->get<uint16_t>("Image", "Offset_X", NULL);
	cout << "\tOffsetY_";
	int16_t offsetY = vision3D->get<uint16_t>("Image", "Offset_Y", NULL);

	centerX	= width/2 + offsetX;
	centerY	= height/2 + offsetY;

	init();

	cout << "End ScanLineHelper3D Constructor" << endl;
}


/*
** Initialization
** Generate the lines and the offsets of them.
** linesX structure is [x1,y1,x2,y2,..]
** linesXOffsets points on the start of each line in linesX
*/
void ScanLineHelper3D::init()
{
	double startX	= 0.0;
	double startY	= 0.0;
	double endX	= 0.0;
	double endY	= 0.0;

	double middleCorrectur = -0.5;

	// "lines" Structure: x,y
	linesInner	= new uint16_t [nLines * maxPoints * 2];
	linesOuter	= new uint16_t [nLines * maxPoints * 2];
	// Offset in "lines" to the startpoint of the specified line.
	linesInnerOffsets	= new uint32_t [nLines + 1];
	linesOuterOffsets	= new uint32_t [nLines + 1];

	uint32_t innerOffsetCounter = 0;
	uint32_t outerOffsetCounter = 0;
	uint16_t offset;
	uint16_t * line;

	holders = new double [6];

	holders[0]	= grad2rad(358.2);	// FirstStart
	holders[1]	= grad2rad(2.1);	// FirstEnd
	holders[2]	= grad2rad(117.9);	// SecondStart
	holders[3]	= grad2rad(122.0);	// SecondEnd
	holders[4]	= grad2rad(237.9);	// ThirdStart
	holders[5]	= grad2rad(242.0);	// ThirdEnd


	for(uint16_t i = 0; i < nLines; i++)
	{
		double angle = 1.0 * i * 2 * M_PI / nLines;
		double borderWidth = 7.0;
		double border1 = 0.0;
		double border2 = 0.0;

		if(dir!=400)
		{
			if(dir<350)
			{
				if(dir>borderWidth)
				{
					border1 = (dir-borderWidth)*M_PI/180;
					border2 = (dir+borderWidth)*M_PI/180;
				}
				else
				{
					border1 = (dir+borderWidth)*M_PI/180;
					border2 = (dir-borderWidth+360)*M_PI/180;
				}
			}
			else
			{
				border1 = (dir+borderWidth-360)*M_PI/180;
				border2	= (dir-borderWidth)*M_PI/180;
			}
		}

		// Leave out the titan holders
		if( (dir==400 && ((angle > holders[0] || angle < holders[1]) ||
			(angle > holders[2] && angle < holders[3]) ||
			(angle > holders[4] && angle < holders[5]) )) ||
			(dir<350 && dir>borderWidth  && ((angle > border2) || (angle < border1))) ||
			((dir>=350 || dir<=borderWidth) && dir!=400 && (angle>border1) && (angle<border2)) )

		{
			linesInnerOffsets[i] = innerOffsetCounter;
			linesOuterOffsets[i] = outerOffsetCounter;
			continue;
		}

		/// Inner lines
		line = linesInner + innerOffsetCounter;
		linesInnerOffsets[i] = innerOffsetCounter;

		// Beginpoint
		startX	= cos(angle) * iRadiusStart + centerX + middleCorrectur;
		startY	= sin(angle) * iRadiusStart + centerY + middleCorrectur;

		// Endpoint
		endX	= cos(angle) * iRadiusEnd + centerX + middleCorrectur;
		endY	= sin(angle) * iRadiusEnd + centerY + middleCorrectur;

		// Fill the line with the points and get the offset.
		offset = ScanLineHelper3D::GetLinePoints(line, startX, startY, endX, endY);
		innerOffsetCounter += 2*offset;

		/// Outer Lines
		line = linesOuter + outerOffsetCounter;
		linesOuterOffsets[i] = outerOffsetCounter;

		// Beginpoint
		for (uint16_t j=0; j<width; j++)
		{
			startX	= cos(angle) * (oRadiusStart-j) + centerX + middleCorrectur;
			startY	= sin(angle) * (oRadiusStart-j) + centerY + middleCorrectur;
			if ( (round(startX) < width) && (round(startY) < height) && (round(startX) > 0) && (round(startY) > 0) )
				break;
		}

		// Endpoint
		endX	= cos(angle) * oRadiusEnd + centerX + middleCorrectur;
		endY	= sin(angle) * oRadiusEnd + centerY + middleCorrectur;

		// Fill the line with the points and get the offset.
		offset = ScanLineHelper3D::GetLinePoints(line, startX, startY, endX, endY);
		outerOffsetCounter += 2*offset;
	}

	// Save the last offsets
	linesInnerOffsets[nLines] = innerOffsetCounter;
	linesOuterOffsets[nLines] = outerOffsetCounter;
}


uint16_t ScanLineHelper3D::GetLinePoints(uint16_t * &line, double &startX, double &startY, double &endX, double &endY)
{
	double	dx	= endX - startX;	// Delta X
	double	dy	= endY - startY;	// Delta Y
	double	stepX	= 0.5;	// Step/Direction
	double	stepY	= 0.5;	// Step/Direction
	double	D	= 0;	// Helper variable
	uint16_t	counter	= 0;	// Pointcounter

	// Debug
// 	printf("ax: %f ay: %f ex: %f ey: %f\n", startX, startY, endX, endY);

	// Get positiv deltas and maybe correct the step direction.
	if(dx < 0.0)	{	stepX = -0.5;	dx = -dx;	}
	if(dy < 0.0)	{	stepY = -0.5;	dy = -dy;	}

	double DX = 2 * dx;	// Helper variable
	double DY = 2 * dy;	// Helper variable

	// Check for better way to draw the line.
	if(dy>dx)
	{
		while(1)
		{
			// Check for image border
			if(round(startX) >= 0 && round(startX) < width && round(startY) >= 0 && round(startY) < height)
			{
				*line++ = round(startX);
				*line++ = round(startY);
				counter++;
			}
			else
			{
				//printf("out of image!!!!!!!!!!!!!!!!!!!!!!!!!!!!1\n");
				break;
			}

			// Check for finishing the line.
			if(round(startY) == round(endY))	break;

			// Take a step to Y
			startY += stepY;

			// Check for a step to X
			D += DX;
			if(round(D) > round(dy))
			{
				startX += stepX;
				D -= DY;
			}
		}
	}
	else
	{
		while(1)
		{
			// Check for image border.
			if(round(startX) >= 0 && round(startX) < width && round(startY) >= 0 && round(startY) < height)
			{
				*line++ = round(startX);
				*line++ = round(startY);
				counter++;
			}
			else
			{
				//printf("out of image!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
				break;
			}

			// Check for finishing the line.
			if(round(startX) == round(endX))	break;

			// Take a step to X.
			startX += stepX;

			// Check for a step to Y
			D += DY;
			if(round(D) > round(dx))
			{
				startY += stepY;
				D -= DX;
			}
		}
	}

	// Debug
// 	std::cout << "Size of line: " << counter << std::endl;

	return counter;
}

double ScanLineHelper3D::grad2rad(double grad)	{	return grad * M_PI / 180;	}

uint16_t * ScanLineHelper3D::getLinesInner() const	{	return linesInner;	}
uint16_t * ScanLineHelper3D::getLinesOuter() const	{	return linesOuter;	}
uint32_t * ScanLineHelper3D::getLinesInnerOffsets() const	{	return linesInnerOffsets;	}
uint32_t * ScanLineHelper3D::getLinesOuterOffsets() const	{	return linesOuterOffsets;	}

double * ScanLineHelper3D::getHolders() const	{	return holders;	}

uint16_t ScanLineHelper3D::getNumberLines() const	{	return nLines;	}
uint16_t ScanLineHelper3D::getMaxPoints() const	{	return maxPoints;	}
uint16_t ScanLineHelper3D::getInnerRadiusStart() const	{	return iRadiusStart;	}
uint16_t ScanLineHelper3D::getInnerRadiusEnd() const	{	return iRadiusEnd;	}
uint16_t ScanLineHelper3D::getOuterRadiusStart() const	{	return oRadiusStart;	}
uint16_t ScanLineHelper3D::getOuterRadiusEnd() const	{	return oRadiusEnd;	}



ScanLineHelper3D::~ScanLineHelper3D()
{
	std::cout << "Destructor of ScanLineHelper3D" << std::endl;

	if( linesInner != NULL )			delete[] linesInner;
	if( linesOuter != NULL )			delete[] linesOuter;
	if( linesInnerOffsets != NULL )	delete[] linesInnerOffsets;
	if( linesOuterOffsets != NULL )	delete[] linesOuterOffsets;
	if( holders != NULL )			delete[] holders;
}

