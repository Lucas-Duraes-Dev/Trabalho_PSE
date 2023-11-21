/*
 * SERVOMOTOR.c
 *
 *  Created on: Oct 6, 2023
 *      Authors: Stephanie, Lucas e Jackson
 */

#include "SERVOMOTOR.h"


/* Com a abertura mínima ocorre aos 1.5 ms enquanto a máxima ao 2.3
 * Assim calculando o intervalo e dividindo pelos 180º conseguimos
 * encontrar qual seria o pulso gerado com base no ângulo passado pela
 * função set PWMAngulo.
 * Isso será importante, pois uma vez recebido um valor para o ângulo
 * a partir do magnetômetro, será mais simples movimentar o servo motor
 * corretamente.
 *
 * Para os cálculos consideramos um período de 20ms (frequência de 50Hz).
 *
 * A função implementada no arquivo main.c irá receber ângulos de 0 a 90º
 * por um intervalo de tempo e depois de 90º a 180º, e passá-los como
 * parâmetros pela função definida a baixo para fazer o servo se mover.
 * Assim como foram feitos testes com valores de ângulos variados para
 * confirmar se o comportamento do servo estava correto.
 *
 * Quando o pulso é 94, temos a posição neutra e quando o pulso é
 * 144 temos a posição máxima.
*/


void setPWMAngulo(TIM_HandleTypeDef timer, uint32_t channel, uint16_t period, uint16_t angulo){
	// Função para determinar o pulso
	uint16_t tempo = 20, pulsOK, T;
	double pulso;
	double intervalo = ((2.3-1.5)/180); // aproximadamente 0,0044
	T = period/10;

	pulso = ((((angulo*intervalo)+1.5)/tempo)*T); // Função para determinar o pulso
	pulsOK = (uint16_t)pulso;

	HAL_TIM_PWM_Stop(&timer, channel); // para de gerar PWM
	TIM_OC_InitTypeDef sConfigOC;
	timer.Init.Period = period; // configura a duração do período
	HAL_TIM_PWM_Init(&timer); // reinicia com novos valores de período
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = pulsOK;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&timer, &sConfigOC, channel);
	HAL_TIM_PWM_Start(&timer, channel); // começa a geração de PWM
}
