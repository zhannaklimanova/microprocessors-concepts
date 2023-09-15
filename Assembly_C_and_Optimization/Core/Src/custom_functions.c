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
void findSqrtTester(volatile unsigned long *port, void (*findSqrt)(float32_t in, float32_t *pOut)) {
	double tolerance = 0.00005;
	// small numbers
	for (float32_t i = 0.001; i < 1; i+=0.001) {
		float32_t sqrt;
		findSqrt(i*i, &sqrt);
		double diff = fabs(sqrt*sqrt - i*i);
		if (diff > tolerance) {
			break;
		}
	}
	// medium numbers
	for (float32_t i = 0.1; i < 10; i+=0.1) {
		float32_t sqrt;
		findSqrt(i*i, &sqrt);
		double diff = fabs(sqrt*sqrt - i*i);
		if (diff > tolerance) {
			break;
		}
	}
	// big numbers
	for (float32_t i = 1; i < 100; i+=1) {
		float32_t sqrt;
		findSqrt(i*i, &sqrt);
		double diff = fabs(sqrt*sqrt - i*i);
		if (diff > tolerance) {
			break;
		}
	}
}

void findTranscendental(float32_t omega, float32_t phi, float32_t *x)
{
	float32_t function = 0;
	float32_t function_derivative = 0;
	float32_t xn = 1.0;

	for(uint32_t i=0; i<100; i++)
	{
		function = arm_cos_f32(omega*(xn) + phi) - (xn)*(xn);
		function_derivative = -omega*arm_sin_f32(omega*(xn) + phi)-2*(xn);
		xn = xn - (function / function_derivative);
	}
	*x = xn;
}



