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

const int mixed_team_flag_size = 2;
const int ball_size = 39;
const int opp_size = 5;
const int opp_count = 10;
const int position_size = 5;

struct ballPos
{
    int16_t ballX;
    int16_t ballY;
    int16_t ballZ;
    int16_t ballVX;
    int16_t ballVY;
    int16_t ballVZ;
    uint8_t confidence;
    void append(unsigned char *ptr)
    {
        int16_t arr[] = {ballX, ballY, ballZ, ballVX, ballVY, ballVZ};
        unsigned char *it = (unsigned char *)&arr[0];
        for (int n = 0; n < 3; n++)
        {
            for (int i = n * (ball_size / 3); i < ball_size / 3 - 1; i++)
            {
                if (n == 0)
                    ptr[i] = it[i];
                else
                {
                    ptr[i] = 0x80;
                    i++;
                    ptr[i] = 0x00;
                }
            }
            if (n == 0)
            {
                ptr[(n + 1) * (ball_size / 3) - 1] = confidence;
            }
            else
            {
                ptr[(n + 1) * (ball_size / 3) - 1] = 0;
            }
        }
    }

    void desrializeFromPtr(unsigned char *ptr)
    {
        int16_t *it = (int16_t *)&ptr[0];
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
        confidence = ptr[(ball_size / 3) - 1];
    }
    void print()
    {
        cout << "(" << ballX << ":" << ballY << ":" << ballZ << ") ";
        cout << "(" << ballVX << ":" << ballVY << ":" << ballVZ << ")";
        cout << confidence << " ";
    }
};

struct point
{
    int16_t x;
    int16_t y;
    uint8_t confidence;

    void append(unsigned char *ptr)
    {
        int16_t arr[] = {x, y};
        unsigned char *it = (unsigned char *)&arr[0];
        for (int i = 0; i < opp_size - 1; i++)
        {
            ptr[i] = it[i];
        }
        ptr[opp_size - 1] = confidence;
    }

    void desrializeFromPtr(unsigned char *ptr)
    {
        int16_t *it = (int16_t *)&ptr[0];
        x = *it;
        it++;
        y = *it;
        confidence = ptr[opp_size - 1];
    }

    void print()
    {
        cout << "(" << x << ":" << y << ")"
             << " " << confidence << endl;
    }
};

#endif /* DATASTRUCTURES_H_ */
