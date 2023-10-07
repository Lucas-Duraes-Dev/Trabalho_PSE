/*
 * SERVOMOTOR.c
 *
 *  Created on: Oct 6, 2023
 *      Author: Stephanie, Lucas e Jackson
 */

#include "SERVOMOTOR.h"

/* O servo se movimento de 5 em 5 graus, portanto devemos
 * encontrar valores que sejam múltiplos de 5.
*/
int roundUp(int num) {
    return ((num / 5) + (num % 5 > 0 ? 1 : 0)) * 5;
}

void setPWMAngulo(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, int angulo){
	int anguloOK;
	uint16_t pulse, T, tempo = 20;
	double intervalo = ((2.3-1.5)/180);
	anguloOK = roundUp(angulo);
	T = period/10;

	// Função para determinar o pulso
	pulse = ((((anguloOK*intervalo)+1.5)/tempo)*T);


	HAL_TIM_PWM_Stop(&timer, channel); // para de gerar PWM
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period; // configura a duração do período
	HAL_TIM_PWM_Init(&timer); // reinicia com novos valores de período
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulse;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel); // começa a geração de PWM
}
