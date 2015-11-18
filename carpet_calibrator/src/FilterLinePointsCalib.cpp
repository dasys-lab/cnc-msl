/*
 * $Id: FilterLinePoints.cpp 1935 2007-03-19 19:50:12Z phbaer $
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
#include "FilterLinePointsCalib.h"

#include <algorithm>
#include <fstream>
#include <math.h>
#include <boost/shared_ptr.hpp>

using namespace std;
#define BLOB_UNDEF 100000

FilterLinePointsCalib::FilterLinePointsCalib(int area) :
		Filter(OF_ZERO, area, area)
{

	this->sc = SystemConfig::getInstance();

	MX = area / 2;
	MY = area / 2;

	Configuration *loc = (*this->sc)["Localization"];

	LinePointsThreshold = (unsigned char)loc->get<int>("Localization", "LinePointsThreshold", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("Localization", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("Localization", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("Localization", "MaxLineWidth", NULL);
	OnlyFirstPoint = loc->get<bool>("Localization", "OnlyFirstPoint", NULL);

	Configuration *vision = (*this->sc)["Vision"];

	negRanges[0][0] = vision->get<short>("Vision", "Holder", "NegRange_0_0", NULL);
	negRanges[0][1] = vision->get<short>("Vision", "Holder", "NegRange_0_1", NULL);
	negRanges[1][0] = vision->get<short>("Vision", "Holder", "NegRange_1_0", NULL);
	negRanges[1][1] = vision->get<short>("Vision", "Holder", "NegRange_1_1", NULL);
	negRanges[2][0] = vision->get<short>("Vision", "Holder", "NegRange_2_0", NULL);
	negRanges[2][1] = vision->get<short>("Vision", "Holder", "NegRange_2_1", NULL);

	short nLines = vision->get<short>("Vision", "NumberScanLines", NULL);

	shared_ptr<std::vector<std::string> > holdersBPtr = (*vision).getSections("Vision", "Holder", NULL);
	std::vector<std::string> * holders = holdersBPtr.get();

	addHolders.clear();

	for (unsigned int i = 0; i < holders->size(); i++)
	{

		Holder addHolder;
		addHolder.start = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "Start", NULL);
		addHolder.end = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "End", NULL);
		addHolders.push_back(addHolder);

		printf("Holder: %s %f %f\n", (*holders)[i].c_str(), addHolder.start, addHolder.end);

	}

	angleValidity = (char *)malloc(360);
	for (unsigned int j = 0; j < 360; j++)
	{

		bool validity = true;

		short i = (short)lrint(j / 360.0 * nLines);
		if (i >= 360)
			i = 0;

		if (negRanges[0][0] > nLines / 2 && negRanges[0][1] < nLines / 2)
		{

			if ((i >= negRanges[0][0]) || (i <= negRanges[0][1]) || (i >= negRanges[1][0] && i <= negRanges[1][1])
					|| (i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}
		else
		{

			if ((i >= negRanges[0][0] && i <= negRanges[0][1]) || (i >= negRanges[1][0] && i <= negRanges[1][1])
					|| (i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}

		double lineAngle = 1.0 * j;

		for (unsigned a = 0; a < addHolders.size(); a++)
		{
			if (addHolders[a].start > 180.0 && addHolders[a].end < 180.0)
			{

				if (lineAngle >= addHolders[a].start || lineAngle <= addHolders[a].end)
					validity = false;

			}
			else
			{

				if (lineAngle >= addHolders[a].start && lineAngle <= addHolders[a].end)
					validity = false;

			}

		}

		angleValidity[j] = validity ? 1 : 0;

	}

	init();

}

FilterLinePointsCalib::FilterLinePointsCalib(int width, int height) :
		Filter(OF_ZERO, width, height)
{

	this->sc = SystemConfig::getInstance();

	Configuration *vision = (*this->sc)["Vision"];
	Configuration *loc = (*this->sc)["Localization"];

	MX = vision->get<int>("Vision", "CameraMX", NULL);
	MY = vision->get<int>("Vision", "CameraMY", NULL);

	LinePointsThreshold = (unsigned char)loc->get<int>("Localization", "LinePointsThreshold", NULL);
	LinePointsJump = (unsigned char)loc->get<int>("Localization", "LinePointsJump", NULL);
	MinLineWidth = (unsigned char)loc->get<int>("Localization", "MinLineWidth", NULL);
	MaxLineWidth = (unsigned char)loc->get<int>("Localization", "MaxLineWidth", NULL);
	OnlyFirstPoint = loc->get<bool>("Localization", "OnlyFirstPoint", NULL);

	//Configuration *vision = (*this->sc)["Vision"];

	negRanges[0][0] = vision->get<short>("Vision", "Holder", "NegRange_0_0", NULL);
	negRanges[0][1] = vision->get<short>("Vision", "Holder", "NegRange_0_1", NULL);
	negRanges[1][0] = vision->get<short>("Vision", "Holder", "NegRange_1_0", NULL);
	negRanges[1][1] = vision->get<short>("Vision", "Holder", "NegRange_1_1", NULL);
	negRanges[2][0] = vision->get<short>("Vision", "Holder", "NegRange_2_0", NULL);
	negRanges[2][1] = vision->get<short>("Vision", "Holder", "NegRange_2_1", NULL);

	short nLines = vision->get<short>("Vision", "NumberScanLines", NULL);

	shared_ptr<std::vector<std::string> > holdersBPtr = (*vision).getSections("Vision", "Holder", NULL);
	std::vector<std::string> * holders = holdersBPtr.get();

	addHolders.clear();

	for (unsigned int i = 0; i < holders->size(); i++)
	{

		Holder addHolder;
		addHolder.start = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "Start", NULL);
		addHolder.end = vision->get<double>("Vision", "Holder", (*holders)[i].c_str(), "End", NULL);
		addHolders.push_back(addHolder);

		printf("Holder: %s %f %f\n", (*holders)[i].c_str(), addHolder.start, addHolder.end);

	}

	angleValidity = (char *)malloc(360);
	for (unsigned int j = 0; j < 360; j++)
	{

		bool validity = true;

		short i = (short)lrint(j / 360.0 * nLines);
		if (i >= 360)
			i = 0;

		if (negRanges[0][0] > nLines / 2 && negRanges[0][1] < nLines / 2)
		{

			if ((i >= negRanges[0][0]) || (i <= negRanges[0][1]) || (i >= negRanges[1][0] && i <= negRanges[1][1])
					|| (i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}
		else
		{

			if ((i >= negRanges[0][0] && i <= negRanges[0][1]) || (i >= negRanges[1][0] && i <= negRanges[1][1])
					|| (i >= negRanges[2][0] && i <= negRanges[2][1]))
				validity = false;

		}

		double lineAngle = 1.0 * j;

		for (unsigned a = 0; a < addHolders.size(); a++)
		{
			if (addHolders[a].start > 180.0 && addHolders[a].end < 180.0)
			{

				if (lineAngle >= addHolders[a].start || lineAngle <= addHolders[a].end)
					validity = false;

			}
			else
			{

				if (lineAngle >= addHolders[a].start && lineAngle <= addHolders[a].end)
					validity = false;

			}

		}

		angleValidity[j] = validity ? 1 : 0;

	}

	init();

}

FilterLinePointsCalib::~FilterLinePointsCalib()
{

	cleanup();

}

unsigned char * FilterLinePointsCalib::process(unsigned char * src, unsigned int width, unsigned int height,
												ScanLineHelper & helper)
{

	unsigned char * tgt = src;

	short * firstInner = helper.getInnerLines();
	short * nInner = helper.getInnerLinesN();

	short * firstOuter = helper.getOuterLines();
	short * nOuter = helper.getOuterLinesN();

	short maxPoints = helper.getMaxPoints();

	short x;
	short y;
	int floorBrightness = 240;

/////////////////
	/*
	 int linePointInd[5000];
	 int lpCount=0;
	 */
/////////////////
	std::vector<short> LinePointsX;
	LinePointsX.clear();
	std::vector<short> LinePointsY;
	LinePointsY.clear();

	//ofstream fs("taggedLinePoints.txt", fstream::app);
	int aprinted = 0;
	for (short i = 0; i < helper.getNumberLines(); i++)
	{
		aprinted = 0;
		short b = 0;
		short e = 0;

		if (i % 2 == 0)
		{

			short * line = firstInner;

			x = *line++;
			y = *line++;

			short vb = src[x * width + y];

			for (short j = 1; j < (*nInner); j++)
			{
				x = *line++;
				y = *line++;
				int k = i - 45;
				if (k < 0)
					k = k + 180;

				short va = src[x * width + y];

				if (va > vb + LinePointsJump && va > LinePointsThreshold)
				{
					b = j;
					e = j;
				}

				if (va < vb - LinePointsJump && va < floorBrightness)
				{
					e = j;
				}

				if (b > 0 & e - b > MinLineWidth & e - b < MaxLineWidth + 10)
				{

					short indX = firstInner[((e + b) / 2) * 2];
					short indY = firstInner[((e + b) / 2) * 2 + 1];

					double angle = -atan2(1.0 * indY - MY, 1.0 * indX - MX);

					if ((e - b) < 30)
					{

						//tgt[indX*width + indY] = 0;
						//linePointInd[lpCount++] = indX*width + indY;

						angle = -angle;
						if (angle < 0.0)
							angle += 2.0 * M_PI;

						short angleInd = (short)lrint(angle * 360.0 / (2.0 * M_PI));
						if (angleInd >= 360)
							angleInd = 0;
						//printf("AngleInd: %d AngleValidity %d\n", angleInd, angleValidity[angleInd]);					
						if (angleValidity[angleInd])
						{
							LinePointsX.push_back(indX);
							LinePointsY.push_back(indY);

							//Transform Coordinate System (0,0)-Image Center
							double xp = indX - (short)width / (short)2;
							double yp = indY - (short)height / (short)2;

							//Compute Angle and Distance
							if (aprinted == 0)
							{
								//fs << atan2(yp, xp) << "\t";
								angles[i] = atan2(yp, xp);
							}
							double imDist = sqrt(xp * xp + yp * yp);
							/*if(aprinted < 8) {
								distanceSum[i][aprinted] = imDist;
							}*/
							//fs << imDist << "\t";

							//Sort into vector
							if (aprinted < 8 && imDist > 100)
							{
								int ih = i;
								//Search next and last valid index
								int ib = ih - 1;
								if (ib < 0)
									ib += 180;
								for (int n = 0; n < 15; n++)
								{
									if (distanceCount[ib][aprinted] != 0)
										break;
									ib--;
									if (ib < 0)
										ib += 180;
								}
								double beforeV = distanceSum[ib][aprinted] / (double)distanceCount[ib][aprinted];
								if (distanceCount[ib][aprinted] == 0)
									beforeV = 0;

								int in = ih + 1;
								if (in >= 180)
									in = 0;
								for (int n = 0; n < 15; n++)
								{
									if (distanceCount[in][aprinted] != 0)
										break;
									in++;
									if (in >= 180)
										in = 0;
								}
								double nextV = distanceSum[in][aprinted] / (double)distanceCount[in][aprinted];
								if (distanceCount[in][aprinted] == 0)
									nextV = 0;

								//Check distance-difference to last/next index
								if (distanceCount[ib][aprinted] == 0 || abs(imDist - beforeV) < 4.0)
								{
									if (distanceCount[in][aprinted] == 0 || abs(imDist - nextV) < 4.0)
									{
										if (aprinted == 0 || distanceCount[ih][aprinted - 1] == 0
												|| imDist
														> distanceSum[ih][aprinted - 1]
																/ (double)distanceCount[ih][aprinted - 1] + 2)
										{
											distanceCount[ih][aprinted]++;
											distanceSum[ih][aprinted] += imDist;
										}
									}
								} // else aprinted = 20;
							}
							aprinted++;
						}
					}

					//printf("LinePoint: %d %d %d\n", i, b, e);
					//printf("LinePointAngle: %f\n", angle/M_PI * 180);
					//printf("LinePointIndex: %d %d\n", indX - height/2, indY - width/2);

					b = 0;
					e = 0;

				}
				vb = va;
				src[x * width + y] = 255;
			}

			firstInner = firstInner + maxPoints * 2;
			nInner++;

		}
		//if(aprinted!=0) fs << endl;

	}
	//fs.close();

	for (unsigned int i = 0; i < LinePointsX.size(); i++)
	{
		tgt[LinePointsX[i] * width + LinePointsY[i]] = 0;
		ofstream ofs("rawLinePoints.txt", fstream::app);
		double x = LinePointsX[i] - (short)width / (short)2;
		double y = LinePointsY[i] - (short)height / (short)2;
		ofs << atan2(y, x) << " " << sqrt(x * x + y * y) << " " << x << " " << y << endl;
	}
	ofstream cfs("clearedLP.txt");
	for (int i = 0; i < distanceSum.size(); i++)
	{
		int element = (i + 1 + (distanceSum.size() / 2)) % distanceSum.size();
		if (angles[element] == 0)
			continue;
		cfs << angles[element] << " ";
		for (int j = 0; j < distanceSum[element].size(); j++)
		{
			if (distanceSum[element][j] != 0)
			{
				cfs << distanceSum[element][j] << " ";
				//cfs << distanceCount[i][j] << " ";
			}
			else
				break;
//			else cfs << 0 << " ";
		}
		cfs << endl;
	}
	cfs.close();

	printf("FilterLinePoints - Number of LinePoints: %d\n", (int)LinePointsX.size());

/////////////
	/*
	 for(int i=0; i<lpCount; i++) {
	 tgt[linePointInd[i]]=0;
	 }
	 */
/////////////
	return tgt;

}

void FilterLinePointsCalib::init()
{
	vector<double> tmp;
	vector<int> tmpint;
	for (int i = 0; i < 8; i++)
	{
		tmp.push_back(0);
		tmpint.push_back(0);
	}
	for (int i = 0; i < 180; i++)
	{
		distanceSum.push_back(tmp);
		distanceCount.push_back(tmpint);

		//int k=i;//-45;
		//if(k<0) k=k+180;
		//angles.push_back(3.141592*((double)k/180.0));
		angles.push_back(0);
	}
}

void FilterLinePointsCalib::cleanup()
{

}

