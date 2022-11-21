/*
 * hal_nucleo_l476.c
 *
 *  Created on: Oct 14, 2022
 *      Author: gosor
 */

#include "hal_nucleo_l476.h"

void HAL_NUCLEO_L476_INIT(void){
	LL_Init1msTick(4000000);

	S1_GPIO_CLK_ENABLE();

	LL_GPIO_SetPinMode(LD3_GPIO_PORT, LD3_PIN, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetOutputPin(LD3_GPIO_PORT, LD3_PIN);

	LL_GPIO_SetPinMode(S1_GPIO_PORT, S1_PIN, LL_GPIO_MODE_INPUT);
}


void LD3_Set(void){
	LL_GPIO_SetOutputPin(LD3_GPIO_PORT, LD3_PIN);
}

void LD3_Reset(void){
	LL_GPIO_ResetOutputPin(LD3_GPIO_PORT, LD3_PIN);
}

