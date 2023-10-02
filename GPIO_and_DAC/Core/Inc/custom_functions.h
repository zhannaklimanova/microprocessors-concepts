/*
 * custom_functions.h
 *
 *  Created on: Sep 25, 2023
 *      Author: zhanna
 */

#ifndef INC_CUSTOM_FUNCTIONS_H_
#define INC_CUSTOM_FUNCTIONS_H_

#include "main.h"

#define TS_CAL1 (uint16_t*)0x1FFF75A8
#define TS_CAL2 (uint16_t*)0x1FFF75CA
#define VREFINT (uint16_t*)0x1FFF75AA
#define TS_CAL1_TEMP 30
#define TS_CAL2_TEMP 110
#define VREFINT_voltage 3

void buttonLightLED();
void convertV2C(uint32_t voltageTemperature, float *vrefScale,  float *celsiusTemperature);
void getVrefRatio(uint32_t voltageRefint, float *vrefScale);
void delay2K();
void delay8K();
uint32_t temp2frequency(uint32_t frequency);


#endif /* INC_CUSTOM_FUNCTIONS_H_ */
