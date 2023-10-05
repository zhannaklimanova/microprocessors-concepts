/*
 * custom_functions..h
 *
 *  Created on: Sep 25, 2023
 *      Author: zklim
 */

#ifndef INC_CUSTOM_FUNCTIONS_H_
#define INC_CUSTOM_FUNCTIONS_H_

#include "main.h"
#include <stdlib.h>
#include "arm_math.h"


//#define TS_CAL1 (uint16_t)0x1FFF75A8
//#define TS_CAL2 (uint16_t)0x1FFF75CA
//#define VREFINT (uint16_t*)0x1FFF75AA
//#define TS_CAL1_TEMP 30
//#define TS_CAL2_TEMP 110
//#define VREFINT_voltage 3

void buttonLightLED();
void generateSAW(uint32_t *currentSAWValue, uint32_t DACIncrement, uint32_t maxDACAmplitude);
void generateTRIANGLE(uint32_t *currentTRIANGLEValue, uint32_t currentSAWValue, uint32_t maxDACAmplitude);
void generateSIN(uint32_t *SINOutput, float_t *currentSINValue, float_t SINIncrement);

#endif /* INC_CUSTOM_FUNCTIONS_H_ */
