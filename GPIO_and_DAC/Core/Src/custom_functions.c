/*
 * customer_functions.c
 *
 *  Created on: Sep 25, 2023
 *      Author: zklim
 */

#include "custom_functions.h"

/**
 * @brief turns the LED light on after button press and turns it off after button release.
 *        Note that the circuit will send a 1 when button is not pressed and create a short-circuit
 *        and send a 0 when button is pressed - the functionality is adjusted accordingly.
 *
 */
void buttonLightLED()
{
	GPIO_PinState buttonStatus = HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
	if (buttonStatus==GPIO_PIN_RESET) // send 0: button pressed
	{
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
	}
	HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET); // send 1: button not pressed
}

void generateSAW(uint32_t *currentSAWValue, uint32_t DACIncrement, uint32_t maxDACAmplitude)
{
	*currentSAWValue += DACIncrement;
	*currentSAWValue = *currentSAWValue % maxDACAmplitude;
}

void generateTRIANGLE(uint32_t *currentTRIANGLEValue, uint32_t currentSAWValue, uint32_t maxDACAmplitude)
{
	*currentTRIANGLEValue = (uint32_t)abs((int32_t)(2 * currentSAWValue - maxDACAmplitude));
}

void generateSIN(uint32_t *SINOutput, float_t *currentSINValue, float_t SINIncrement)
{
	float_t Amplitude = 100.0; // not 128 because causes clipping
	*SINOutput = (uint32_t)(Amplitude * (arm_sin_f32(*currentSINValue) + 1.0)); // need to cast to put into HAL_DAC_SetValue function
	*currentSINValue += SINIncrement;

	if (*currentSINValue > (2*M_PI))
	{
		*currentSINValue = 0.0;
	}
}
