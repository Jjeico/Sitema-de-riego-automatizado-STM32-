#include "stdio.h"
#include "delay.h"

#ifndef INC_DISTANCIA_H_
#define INC_DISTANCIA_H_
//SENSOR HC-SR04 (PROXIMIDAD)
//Vcc = +5V, Trig = PA9, Echo = PA8

#define TRIG_PIN GPIO_PIN_9
#define TRIG_PORT GPIOA
#define ECHO_PIN GPIO_PIN_8
#define ECHO_PORT GPIOA
uint32_t pMillis;
uint32_t Value1 = 0;
uint32_t Value2 = 0;
uint16_t Distance  = 0;  // cm

TIM_HandleTypeDef htim1;

void DISTANCIA(){
HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_SET);  // pull the TRIG pin HIGH
__HAL_TIM_SET_COUNTER(&htim1, 0);
while (__HAL_TIM_GET_COUNTER (&htim1) < 10);  // wait for 10 us
HAL_GPIO_WritePin(TRIG_PORT, TRIG_PIN, GPIO_PIN_RESET);  // pull the TRIG pin low

pMillis = HAL_GetTick(); // used this to avoid infinite while loop  (for timeout)
// wait for the echo pin to go high
while (!(HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 10 >  HAL_GetTick());
Value1 = __HAL_TIM_GET_COUNTER (&htim1);

pMillis = HAL_GetTick(); // used this to avoid infinite while loop (for timeout)
// wait for the echo pin to go low
while ((HAL_GPIO_ReadPin (ECHO_PORT, ECHO_PIN)) && pMillis + 50 > HAL_GetTick());
Value2 = __HAL_TIM_GET_COUNTER (&htim1);

Distance = (Value2-Value1)* 0.034/2;
}



#endif /* INC_DISTANCIA_H_ */
