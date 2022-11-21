#include "stdio.h"
#include "delay.h"


#ifndef INC_HUMEDAD_H_
#define INC_HUMEDAD_H_

//SENSOR HUMEDAD
//Vcc = +5V, AO = PC4

uint16_t readValue =0;
ADC_HandleTypeDef hadc1;

void HUMEDAD(){
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1,1000);
    readValue = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
}


#endif /* INC_HUMEDAD_H_ */
