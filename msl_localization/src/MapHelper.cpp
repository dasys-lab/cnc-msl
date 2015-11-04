#include "MapHelper.h"
#include <FootballField.h>
#include <global/CoxTypes.h>
#include <helpers/Line2D.h>

using namespace std;
using namespace Cox;

MapHelper* MapHelper::instance = NULL;

MapHelper* MapHelper::getInstance() {
	if (instance == NULL)
		instance = new MapHelper();
	return instance;
}

MapHelper::MapHelper() {
	FootballField::getInstance();
	derivation = true;
}

void MapHelper::initializeMap() {
	//This is making a copy of the list -> shit!
	auto circles = getCircles();
	auto lines = getLines();
	//Note this should be max/min double
	__max_x = -9999999;
	__max_y = -9999999;
	__min_x = 9999999;
	__min_y = 9999999;

	field_lines_t::const_iterator it = lines.begin();
	for (; it != lines.end(); ++it) {
		Line2D line = *it;
		__max_x = std::max(__max_x, line.get_start().x);
		__max_y = std::max(__max_y, line.get_start().y);
		__min_x = std::min(__min_x, line.get_start().x);
		__min_y = std::min(__min_y, line.get_start().y);

		__max_x = std::max(__max_x, line.get_end().x);
		__max_y = std::max(__max_y, line.get_end().y);
		__min_x = std::min(__min_x, line.get_end().x);
		__min_y = std::min(__min_y, line.get_end().y);
	}

	float border = 1000.f; //Maybe load from Globals.conf, but it has never been loaded from there before and worked with 600
	__min_x -= border;
	__max_x += border;
	__min_y -= border;
	__max_y += border;
	RESOLUTION = 10;

	WIDTH = std::floor((__max_x - __min_x) / RESOLUTION) + 1;
	HEIGHT = std::floor((__max_y - __min_y) / RESOLUTION) + 1;
	IHEIGHT = HEIGHT;
	IWIDTH = WIDTH;
	const int MAPSIZE = (WIDTH)*(HEIGHT);
	cout << "Mapwidth: " << WIDTH << endl;
	cout << "Mapheight: " << HEIGHT << endl;
	cout << "Mapsize: " << MAPSIZE << endl;

	newMap = new unsigned char[MAPSIZE];
	memset(newMap, 255,	MAPSIZE * sizeof(unsigned char));

	wallDistanceMap = new float[MAPSIZE];

	for (float y = __min_y; y <= __max_y; y += RESOLUTION) {
		for (float x = __min_x; x <= __max_x; x += RESOLUTION) {
			float min_dist = 1e50;
			float dist = 0.;

			for (field_lines_t::const_iterator it = lines.begin();
					it != lines.end(); it++) {
				dist = it->p2line_distance(x, y);

				if (dist < min_dist) {
					min_dist = dist;
				}
			}

			// check distance to circles
			for (field_circles_t::const_iterator it = circles.begin();
					it != circles.end(); ++it) {
				dist = it->get_distance(x, y);

				if (dist < min_dist) {
					min_dist = dist;
				}
			}

			int mapPosX = std::floor((x - __min_x) / RESOLUTION);
			int mapPosY = std::floor((y - __min_y) / RESOLUTION);
			//cout << id(mapPosX, mapPosY, WIDTH) << " " << flush;

			double tribot = 1 - (250*250/(min_dist*min_dist + 250*250));
			unsigned int tribot_real = (unsigned int) lrint(tribot*255.0);

			newMap[id(mapPosX, mapPosY, WIDTH)] = tribot_real;
			wallDistanceMap[id(mapPosX, mapPosY, WIDTH)] = min_dist;
		}
	}
	initializeGradientMap();


	//This is the minimum and maximum location to sample particles later on
	//Originally this was designed for oversized maps of ROS. In RoboCup it might be even too small.
	minXLocation = 0;
	maxXLocation = WIDTH;
	minYLocation = 0;
	maxYLocation = HEIGHT;


//	ofstream ofs("Test3.txt");
//	//ofstream ofs2("Test2.txt");
//	float distance, ef, derrddist, csquare = 0.25 * 0.25;
//	for (int i = 0; i < IWIDTH * IHEIGHT;
//			i++) {
//		//ofs2 << (int) rmr->getMap()[i] << " ";
//		//ofs2 << (int)ySobelMap[i] << " ";
//
//		distance = wallDistanceMap[i];
//		ef = csquare + distance * distance;
//		derrddist = (2 * csquare * distance) / (ef * ef);
//
//		//ofs << derrddist * fYSobel[i] << " ";
//		//ofs << distance << " ";
//		ofs << fXSobel[i] << " ";
//		if (i % (int)IWIDTH == (int)IWIDTH - 1) {
//			ofs << endl;
//			//ofs2 << endl;
//		}
//	}
//	ofs.close();
//	//ofs2.close();
	cout << "Map Initialization Finished" << endl;
}

void MapHelper::initializeGradientMap() {
	int WIDTH = this->WIDTH;
	int HEIGHT = this->HEIGHT;

	xSobelMap = new signed char[WIDTH * HEIGHT * sizeof(signed char)];
	memset(xSobelMap, 0, WIDTH * HEIGHT * sizeof(signed char));

	ySobelMap = new signed char[WIDTH * HEIGHT * sizeof(signed char)];
	memset(ySobelMap, 0, WIDTH * HEIGHT * sizeof(signed char));

	{
		int slt, slc, slb, sct, scc, scb, srt, src, srb;
		for (int x = 1; x < WIDTH - 1; x++) {
			for (int y = 1; y < HEIGHT - 1; y++) {
				slt = newMap[id(x - 1, y - 1, WIDTH)];
				slc = newMap[id(x - 1, y, WIDTH)];
				slb = newMap[id(x - 1, y + 1, WIDTH)];
				sct = newMap[id(x, y - 1, WIDTH)];
				scc = newMap[id(x, y, WIDTH)];
				scb = newMap[id(x, y + 1, WIDTH)];
				srt = newMap[id(x + 1, y - 1, WIDTH)];
				src = newMap[id(x + 1, y, WIDTH)];
				srb = newMap[id(x + 1, y + 1, WIDTH)];

				if (scc == 0) {
					xSobelMap[id(x, y, WIDTH)] = 0;
					ySobelMap[id(x, y, WIDTH)] = 0;
				} else {
					xSobelMap[id(x, y, WIDTH)] = -(slt + 2 * slc + slb - srt
							- 2 * src - srb) / 8;
					ySobelMap[id(x, y, WIDTH)] = -(slt + 2 * sct + srt - slb
							- 2 * scb - srb) / 8;
				}
			}
		}
	}

	fXSobel = new float[WIDTH * HEIGHT];
	fYSobel = new float[WIDTH * HEIGHT];
	float RES = RESOLUTION;
	{
		float slt, slc, slb, sct, scc, scb, srt, src, srb;
		for (int x = 1; x < WIDTH - 1; x++) {
			for (int y = 1; y < HEIGHT - 1; y++) {
				slt = wallDistanceMap[id(x - 1, y - 1, WIDTH)];
				slc = wallDistanceMap[id(x - 1, y, WIDTH)];
				slb = wallDistanceMap[id(x - 1, y + 1, WIDTH)];
				sct = wallDistanceMap[id(x, y - 1, WIDTH)];
				scc = wallDistanceMap[id(x, y, WIDTH)];
				scb = wallDistanceMap[id(x, y + 1, WIDTH)];
				srt = wallDistanceMap[id(x + 1, y - 1, WIDTH)];
				src = wallDistanceMap[id(x + 1, y, WIDTH)];
				srb = wallDistanceMap[id(x + 1, y + 1, WIDTH)];

				if (scc == 0) {
					fXSobel[id(x, y, WIDTH)] = 0;
					fYSobel[id(x, y, WIDTH)] = 0;
				} else {
					fXSobel[id(x, y, WIDTH)] = -((slt + 2 * slc + slb - srt
							- 2 * src - srb) / 8.0) / RES;
					fYSobel[id(x, y, WIDTH)] = -((slt + 2 * sct + srt - slb
							- 2 * scb - srb) / 8.0) / RES;
				}
			}
		}
	}

}


double MapHelper::fxGradient(int indX, int indY) {
	return (double) fXSobel[id(indX, indY, IWIDTH)];
}

double MapHelper::fyGradient(int indX, int indY) {
	return (double) fYSobel[id(indX, indY, IWIDTH)];
}

double MapHelper::fangleGradient(int px, int py, double pangle, double lx,
		double ly) {
	double sinangle = sin(pangle);
	double cosangle = cos(pangle);
	return (-sinangle * lx - cosangle * ly) * fxGradient(px, py)
			- (cosangle * lx - sinangle * ly) * fyGradient(px, py);
}


Cox::field_circles_t MapHelper::getCircles() {
	//fieldcircles
	Cox::field_circles_t field_circles;
	//mid circle
	field_circles.push_back(
			Cox::circle_t(FootballField::MiddleCircleRadius, 0, 0));
	//cornercircles
	if (FootballField::CornerCircleExists) {
		field_circles.push_back(
				Cox::circle_t(FootballField::CornerCircleRadius,
						FootballField::FieldLength / 2,
						FootballField::FieldWidth / 2));
		field_circles.push_back(
				Cox::circle_t(FootballField::CornerCircleRadius,
						FootballField::FieldLength / 2,
						-FootballField::FieldWidth / 2));
		field_circles.push_back(
				Cox::circle_t(FootballField::CornerCircleRadius,
						-FootballField::FieldLength / 2,
						-FootballField::FieldWidth / 2));
		field_circles.push_back(
				Cox::circle_t(FootballField::CornerCircleRadius,
						-FootballField::FieldLength / 2,
						FootballField::FieldWidth / 2));
	}
	std::cout << "NewLoc nr of circles " << field_circles.size() << std::endl;
	return field_circles;
}

Cox::field_lines_t MapHelper::getLines() {

	Cox::field_lines_t lines;
	int start_x, start_y, end_x, end_y;

	//field lines
	start_x = FootballField::FieldLength / 2;
	start_y = FootballField::FieldWidth / 2;
	end_x = FootballField::FieldLength / 2;
	end_y = -FootballField::FieldWidth / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = FootballField::FieldLength / 2;
	start_y = -FootballField::FieldWidth / 2;
	end_x = -FootballField::FieldLength / 2;
	end_y = -FootballField::FieldWidth / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = -FootballField::FieldLength / 2;
	start_y = -FootballField::FieldWidth / 2;
	end_x = -FootballField::FieldLength / 2;
	end_y = FootballField::FieldWidth / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = -FootballField::FieldLength / 2;
	start_y = FootballField::FieldWidth / 2;
	end_x = FootballField::FieldLength / 2;
	end_y = FootballField::FieldWidth / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	//middle line
	start_x = 0;
	start_y = FootballField::FieldWidth / 2;
	end_x = 0;
	end_y = -FootballField::FieldWidth / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	//goal left 6
	start_x = -FootballField::FieldLength / 2;
	start_y = -FootballField::GoalInnerAreaLength / 2;
	end_x = -FootballField::FieldLength / 2 + FootballField::GoalInnerAreaWidth;
	end_y = -FootballField::GoalInnerAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));
	//7
	start_x = -FootballField::FieldLength / 2
			+ FootballField::GoalInnerAreaWidth;
	start_y = -FootballField::GoalInnerAreaLength / 2;
	end_x = -FootballField::FieldLength / 2 + FootballField::GoalInnerAreaWidth;
	end_y = FootballField::GoalInnerAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = -FootballField::FieldLength / 2
			+ FootballField::GoalInnerAreaWidth;
	start_y = FootballField::GoalInnerAreaLength / 2;
	end_x = -FootballField::FieldLength / 2;
	end_y = FootballField::GoalInnerAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	//goal right
	start_x = FootballField::FieldLength / 2;
	start_y = -FootballField::GoalInnerAreaLength / 2;
	end_x = FootballField::FieldLength / 2 - FootballField::GoalInnerAreaWidth;
	end_y = -FootballField::GoalInnerAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = FootballField::FieldLength / 2
			- FootballField::GoalInnerAreaWidth;
	start_y = -FootballField::GoalInnerAreaLength / 2;
	end_x = FootballField::FieldLength / 2 - FootballField::GoalInnerAreaWidth;
	end_y = FootballField::GoalInnerAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = FootballField::FieldLength / 2
			- FootballField::GoalInnerAreaWidth;
	start_y = FootballField::GoalInnerAreaLength / 2;
	end_x = FootballField::FieldLength / 2;
	end_y = FootballField::GoalInnerAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	//penaltyleft

	start_x = -FootballField::FieldLength / 2;
	start_y = -FootballField::GoalAreaLength / 2;
	end_x = -FootballField::FieldLength / 2 + FootballField::GoalAreaWidth;
	end_y = -FootballField::GoalAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));
	//7
	start_x = -FootballField::FieldLength / 2 + FootballField::GoalAreaWidth;
	start_y = -FootballField::GoalAreaLength / 2;
	end_x = -FootballField::FieldLength / 2 + FootballField::GoalAreaWidth;
	end_y = FootballField::GoalAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = -FootballField::FieldLength / 2 + FootballField::GoalAreaWidth;
	start_y = FootballField::GoalAreaLength / 2;
	end_x = -FootballField::FieldLength / 2;
	end_y = FootballField::GoalAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	//penalty right
	start_x = FootballField::FieldLength / 2;
	start_y = -FootballField::GoalAreaLength / 2;
	end_x = FootballField::FieldLength / 2 - FootballField::GoalAreaWidth;
	end_y = -FootballField::GoalAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = FootballField::FieldLength / 2 - FootballField::GoalAreaWidth;
	start_y = -FootballField::GoalAreaLength / 2;
	end_x = FootballField::FieldLength / 2 - FootballField::GoalAreaWidth;
	end_y = FootballField::GoalAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	start_x = FootballField::FieldLength / 2 - FootballField::GoalAreaWidth;
	start_y = FootballField::GoalAreaLength / 2;
	end_x = FootballField::FieldLength / 2;
	end_y = FootballField::GoalAreaLength / 2;
	std::cout << "NewLoc (" << start_x << "," << start_y << ") " << "(" << end_x
			<< "," << end_y << ") " << std::endl;
	lines.push_back(Cox::Line2D(start_x, start_y, end_x, end_y));

	std::cout << "NewLoc map lines " << lines.size() << std::endl;
	return lines;

}
