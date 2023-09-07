/*
 * cmax.c
 *
 *  Created on: Sep 6, 2023
 *      Author: zhanna
 */

#include "lab1math.h"

void cMax(float *array, uint32_t size, float *max, uint32_t *maxIndex)
{
	(*maxIndex) = 0;
	(*max) = 0;

	for (uint32_t i=0; i<size; i++)
	{
		if (array[i] > (*max))
		{
			(*max) = array[i];
			(*maxIndex) = i;
		}
	}
}

void sqrtNewtonRaphson(float *in, float *pOut)
{
	float tolerance = 0.0001;
	float xn = *in / 2.0; // initial guess
	float difference = 0;

    while(1)
    {
        (*pOut) = 0.5 * (xn + (*in / xn));
        difference = xn - *pOut;
        if (difference < 0)
        {
        	difference = -difference;
        }
        if (difference < tolerance) break;
        xn = (*pOut);
    }

}
