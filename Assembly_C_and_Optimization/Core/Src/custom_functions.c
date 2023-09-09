/*
 * custom_functions.c
 *
 *  Created on: Sep 5, 2023
 *      Author: zhanna
 */
#include "custom_functions.h"

void findMax(float *array, uint32_t size, float *max, uint32_t *maxIndex)
{
	*max = array[0];
	*maxIndex = 0;

	for (uint32_t i=1; i<size; i++)
	{
		if (array[i]>*max)
		{
			*max = array[i];
			*maxIndex = i;
		}
	}
}

void findSqrt(float32_t in, float32_t *pOut)
{
	float32_t tolerance = 0.000001;
	*pOut = in;

	while(((*pOut)*(*pOut)-in)>tolerance)
	{
		*pOut = 0.5 * (*pOut + in / *pOut);
	}
}


