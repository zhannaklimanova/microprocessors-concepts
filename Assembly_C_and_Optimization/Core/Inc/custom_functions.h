/*
 * custom_functions.h
 *
 *  Created on: Sep 5, 2023
 *      Author: zhanna
 */

#ifndef INC_CUSTOM_FUNCTIONS_H_
#define INC_CUSTOM_FUNCTIONS_H_

#include "main.h"

void findMax(float *array, uint32_t size, float *max, uint32_t *maxIndex);
extern void findMaxAsm(float *array, uint32_t size, float *max, uint32_t *maxIndex);

void findSqrt(float32_t in, float32_t *pOut);
extern void findSqrtAsm(float32_t in, float32_t *pOut);

void findTranscendental(float32_t omega, float32_t phi, float32_t *x);

#endif /* INC_CUSTOM_FUNCTIONS_H_ */
