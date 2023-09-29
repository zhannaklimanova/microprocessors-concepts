/*
 * custom_functions.c
 *
 *  Created on: Sep 25, 2023
 *      Author: zhanna
 */

#include "custom_functions.h"

void buttonLightLED()
{
	  GPIO_PinState buttonRead = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
	  switch(buttonRead)
	  {
		  case GPIO_PIN_SET:
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
			  break;
		  case GPIO_PIN_RESET:
			  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
			  break;
	  }
}

void convertV2C(uint32_t voltageTemperature, float *vrefScale,  float *celsiusTemperature)
{

	int32_t dTEMP = TS_CAL2_TEMP - TS_CAL1_TEMP;
	int32_t dCAL = *TS_CAL2 - *TS_CAL1;
	float conversionFactor = (float) dTEMP / (float) dCAL;
	int32_t diff = (float)voltageTemperature * (*vrefScale)-(int32_t)(*TS_CAL1);
	*celsiusTemperature = conversionFactor * (float) diff + 30.0;
}

void getVrefRatio(uint32_t voltageRefint, float *vrefScale)
{
	float vref = (float)(VREFINT_voltage * (*VREFINT))/(float)voltageRefint;
	*vrefScale = vref / (float)VREFINT_voltage;
}

