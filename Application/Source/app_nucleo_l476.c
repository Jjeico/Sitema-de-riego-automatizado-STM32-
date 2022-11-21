/*
 * app_nucleo_476.c
 *
 *  Created on: Oct 14, 2022
 *      Author: gosor
 */

#include "app_nucleo_l476.h"

typedef enum {UP,DOWN} push_button_t;


extern uint32_t Get_Tick(void);

void FSM_Read_PushButton_1(uint8_t *flag_1){
	static uint32_t last_tick_1 = 0;
	static push_button_t button_state_1  = UP;

	uint32_t current_tick_1;
	uint32_t idr_1;
	uint8_t A1;



	current_tick_1 = Get_Tick();
	if (current_tick_1-last_tick_1 >= 100){
		last_tick_1 = current_tick_1;
		idr_1 = LL_GPIO_ReadInputPort(S2_GPIO_PORT);
		idr_1 &= S2_PIN;
		(idr_1 > 0) ?	(A1 = 1U) : (A1 = 0U);

		if (button_state_1 == UP){
			if (A1 == 0)
				button_state_1 = DOWN;
		}else{
			if (A1 == 1){
				button_state_1 = UP;
				if (*flag_1 == 1)
					*flag_1=0;
				else
					*flag_1=1;

			}
		}
	}
}


void DireccionalDerecha(uint8_t *flag_1){


	if (*flag_1 == 1)
	{
		LD3_Set();

	}else{
		if (*flag_1 == 0){
			LD3_Reset();
			}
		}
	}


