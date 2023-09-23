/*
 * custom_functions.c
 *
 *  Created on: Sep 5, 2023
 *      Author: zhanna
 */
#include "custom_functions.h"

/**
 * @brief Finds the maximum value in the array and updates the maximum value and its associated
 * 		  index location.
 *
 * @param array
 * @param size
 * @param max
 * @param maxIndex
 */
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

/**
 * @brief Finds square root of the input value using the Newton-Raphson method.
 *
 * @param in
 * @param out
 */
void findSqrt(float32_t in, float32_t *pOut)
{
	float32_t tolerance = 0.000001;

	if (in < 0) {
		*pOut = 0.0 / 0.0;
		return;
	}

	*pOut = in; // set initial value

	while(((*pOut)*(*pOut)-in)>tolerance)
	{
		*pOut = 0.5 * (*pOut + in / *pOut);
	}
}

/**
 * @brief Finds a solution to x^2 = cosine(omega(x) +  phi) using the Newton-Raphson method.
 *
 * @param omega
 * @param phi
 * @param x
 */
void findTranscendental(float32_t omega, float32_t phi, float32_t *x) {
	const int MAX_ITERATION = 100; // good enough after some iterations
	float32_t tolerance = 0.00001;

	float32_t function = 0;
	float32_t function_derivative = 0;
	float32_t xn = 0.0;

	for (uint32_t i = 0; i < MAX_ITERATION; i++) {
		function = arm_cos_f32(omega * (xn) + phi) - (xn) * (xn);
		function_derivative = -omega * arm_sin_f32(omega * (xn) + phi) - 2 * (xn);
		xn = xn - (function / function_derivative);

		// keep xn within the bound [-1.5, 1.5] because we never adjust the vertical position
		if (xn > 1.5) {
			// pseudo-pseudo random, ensure a whole range of values are tested
			xn = (float32_t)i / (float32_t)MAX_ITERATION;
		} else if (xn < -1.5) {
			// pseudo-pseudo random, ensure a whole range of values are tested
			xn = -(float32_t)i / (float32_t)MAX_ITERATION;
		}
	}
	// Outside the tolerated range, return nan otherwise return solution
	if (function > tolerance || function < -tolerance) {
		*x = 0.0 / 0.0;
	} else {
		*x = xn;
	}
}
