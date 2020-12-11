/*
 * my_math.c
 *
 *  Created on: 2020年12月10日
 *      Author: liuhe
 */

#include "my_math.h"
#include <stdio.h>
#include <stdlib.h>

#define PI 3.1415926

int my_int_abs(int input)
{
    if (input < 0)
        return (-input);
    else
        return input;
}

float my_float_abs(float input)
{
    if (input < 0)
        return (-input);
    else
        return input;
}

int my_max(int x1, int x2)
{
    if (x1 > x2)
        return x1;
    else
        return x2;
}

int my_min(int x1, int x2)
{
    if (x1 > x2)
        return x2;
    else
        return x1;
}

float my_pow(int  base, int power)
{
    float result = 1;
    if (power >= 2)
    {
        for (int i = 0; i < power; i++)
        {
            result = result * base;

        }
        return result;
    }
    else
    {
        return false;
    }
}

float my_arctan(float slope)
{
    float angel;
    if ((slope > -1) && (slope < 1))
    {
        angel = slope - my_pow(slope, 3) / 3 + my_pow(slope, 5) / 5 - my_pow(slope, 7) / 7 + my_pow(slope, 9) / 9;
    }

    else
    {
        slope = 1 / slope;
        angel = slope - my_pow(slope, 3) / 3 + my_pow(slope, 5) / 5 - my_pow(slope, 7) / 7 + my_pow(slope, 9) / 9;
        angel = PI / 2 - angel;
    }
    angel = 180*angel/ PI;
    return angel;
}

int two_point_distance2(int x1, int y1, int x2, int y2)
{
    int distance2 = (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2);
    return distance2;
}




