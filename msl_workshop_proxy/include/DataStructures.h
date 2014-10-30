/*
 * DataStructures.h
 *
 *  Created on: 30.10.2014
 *      Author: endy
 */

#ifndef DATASTRUCTURES_H_
#define DATASTRUCTURES_H_

#include <iostream>

using namespace std;

const int mixed_team_flag_size = 1;
const int ball_size = 36;
const int opp_size = 4;
const int opp_count = 10;
const int position_size = 4;



struct ballPos
{
	int16_t ballX;
	int16_t ballY;
	int16_t ballZ;
	int16_t ballVX;
	int16_t ballVY;
	int16_t ballVZ;
	void append(unsigned char* ptr)
	{
		int16_t arr[] = {ballX, ballY, ballZ, ballVX, ballVY, ballVZ};
		unsigned char* it = (unsigned char*)&arr[0];
		for (int i = 0; i < ball_size; i++)
		{
			ptr[i] = it[i];
		}
	}

	void desrializeFromPtr(unsigned char* ptr)
	{
		int16_t* it = (int16_t*)&ptr[0];
		ballX = *it;
		it++;
		ballY = *it;
		it++;
		ballZ = *it;
		it++;
		ballVX = *it;
		it++;
		ballVY = *it;
		it++;
		ballVZ = *it;
		it++;
	}
	void print() {
		cout << "(" << ballX << ":" << ballY << ":" << ballZ << ") ";
		cout << "(" << ballVX << ":" << ballVY << ":" << ballVZ << ")";
	}
};

struct point
{
	int16_t x;
	int16_t y;
	void append(unsigned char* ptr)
	{
		int16_t arr[] = {x, y};
		unsigned char* it = (unsigned char*)&arr[0];
		for (int i = 0; i < opp_size; i++)
		{
			ptr[i] = it[i];
		}
	}

	void desrializeFromPtr(unsigned char* ptr)
	{
		int16_t* it = (int16_t*)&ptr[0];
		x = *it;
		it++;
		y = *it;
	}

	void print() {
		cout << "(" << x << ":" << y << ")";
	}
};



#endif /* DATASTRUCTURES_H_ */
