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

	if (in < 0) {
		*pOut = 0.0 / 0.0;
		return;
	}

	*pOut = in;

	while(((*pOut)*(*pOut)-in)>tolerance)
	{
		*pOut = 0.5 * (*pOut + in / *pOut);
	}
}

void findTranscendental(float32_t omega, float32_t phi, float32_t *x)
{
	float32_t xn = 1.0;

	float32_t function = 0;
	float32_t function_derivative = 0;

	const int max_iteration = 100;

	float32_t delta = 0;
	const float32_t delta_mult = omega != 0.0 ? PI / omega : PI;
	const float32_t max_delta = delta_mult/max_iteration;
	const float32_t min_delta = -delta_mult/max_iteration;

	for(uint32_t i=0; i<max_iteration; i++)
	{
		function = arm_cos_f32(omega*(xn) + phi) - (xn)*(xn);
		function_derivative = -omega*arm_sin_f32(omega*(xn) + phi)-2*(xn);

		// limit delta
		delta = function / function_derivative;
		delta = (delta > max_delta) ? max_delta : delta;
		delta = (delta < min_delta) ? min_delta : delta;

		if (function_derivative == 0.0) {
			break;
		}

		xn = xn - delta;
	}
	*x = xn;
}

