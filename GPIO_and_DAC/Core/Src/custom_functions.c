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
