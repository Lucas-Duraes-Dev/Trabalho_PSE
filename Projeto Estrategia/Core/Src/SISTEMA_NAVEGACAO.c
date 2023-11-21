/*
 * SISTEMA_NAVEGACAO.c
 *
 *  Created on: Nov 19, 2023
 *      Author: Stephanie, Lucas e Jackson
 */

#include "SISTEMA_NAVEGACAO.h"
#include "MAGNETOMETRO.h"
#include "SERVOMOTOR.h"
#include "L293D.h"
#include "bluetooth.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;

// O valor de RSSI é obtido a partir do Módulo Bluetooth
Coordenadas encontrarPosicao(float RSSI_1, float RSSI_2, float RSSI_3) {

  int8_t P = -69; // Valor de potência (geralmente com valor de -69)
  int8_t N = 2;  // NI

  // A distância é dada pela equação d = 10^((P-RSSI)/(10*N))
  float d1 = pow(10,((P - RSSI_1)/(10*N)));
  float d2 = pow(10,((P - RSSI_2)/(10*N)));
  float d3 = pow(10,((P - RSSI_3)/(10*N)));

  // Posição dos Beacons ao redor do Lago da Reitoria
  // Em coordenadas geográficas
  const Coordenadas B1 = {-19.866733, -43.964666}; // Chegada
  const Coordenadas B2 = {-19.866425, -43.964556}; // Partida
  const Coordenadas B3 = {-19.866581, -43.964787};   // Apoio

  // Encontrar a posição dos pontos X e Y
  // x = (CE - FB)/(EA - BD)
  // y = (CD - AF)/(BD - AE)
  float A = (-2*B1.x + 2*B2.x);
  float B = (-2*B1.y + 2*B2.y);
  float C = pow(d1,2) - pow(d2,2) - pow(B1.x,2) + pow(B2.x,2) - pow(B1.y,2) + pow(B2.y,2);
  float D = (-2*B2.x + 2*B3.x);
  float E = (-2*B2.y + 2*B3.y);
  float F = pow(d2,2) - pow(d3,2) - pow(B2.x,2) + pow(B3.x,2) - pow(B2.y,2) + pow(B2.y,2);

  Coordenadas c = {
    .x = ((C*E) - (F*B)) / ((E*A) - (B*D)),
    .y = ((C*D) - (F*A)) / ((B*D) - (A*E))
  };

  return c;
}

void pilotoAutomatico(){


	/* Será utilizada uma Máquina de estados para
	 * representar os comportamentos que o barco deve
	 * ter em cada um das situações
	 */
	switch(estado){
		case 0:
			// Estado Inicial - o barco está parado
			estadoInicial();
		break;
		case 1:
			// Enquanto se define o ângulo e o servo está direcionando
			// o barco, é melhor que ele se alinhe a uma velocidade menor
			proximoDestino();
		break;
		case 2:
			// Define uma regulação da velocidade do barco enquanto ele se move em linha reta
			linhaReta();
		break;
		case 3:
			retaFinal();
		break;
		default:
		break;
	}
}

void estadoInicial(){
	velocidade = 0;
	angulo = 0;
	n = 0;
	if(posicaoAtual = encontrarPosicao(RSSI_1, RSSI_2, RSSI_3) != 0){
		estado = 1;
	}

}

void proximoDestino(){
	velocidade = 30;
	posicaoAtual = encontrarPosicao(RSSI_1, RSSI_2, RSSI_3);

	// Caso tenha ocorrido um erro e o barco já tenha passado pelo ponto de Apoio passar para o próximo ponto
	if(abs(vetorPontosApoio[n].x) > abs(posicaoAtual.x) || abs(vetorPontosApoio[n].x) > abs(posicaoAtual.x)){
		n++;
	}

	A = (vetorPontosApoio[n].x - posicaoAtual.x);
	C = (vetorPontosApoio[n].y - posicaoAtual.y);
	angulo = (uint16_t) ((180/3.14)*atan2(A,C));

	// O ângulo que deve ser passado ao servo motor é o contrário a esse
	angulo = -1 * angulo;

	/* Como dentre as forças que atuam no barco, o vento
	 * será uma componente nele. Sabendo dessa informação e de que
	 * a direção do vento dominante para a região da Pampulha
	 * vai no sentido Leste --> Nordeste, conforme pudemos verificar
	 * no site que oferece a direção dos ventos
	 * do Aeroporto da Pampulha.
	 * //https://pt.windfinder.com/windstatistics/belo_horizonte_pampulha
	 * Para corrigi-lo será adicionado um offset.
	*/

	// envia ângulo ao Servo
	setPWMAngulo(htim3, TIM_CHANNEL_2, 12500, angulo+offset);

	// Liga o motor DC, a uma velocidade não muito alta para não atrapalhar no alinhamento
	changeMotorSpeed(motor_dc, velocidade);

	// Se já estiver alinhado, vamos para o próximo estado da Máquina de Estados
	if((angulo == 0) || (angulo <= 5)){
		estado = 2;
	}

	if(n == numeroPontos - 1){
		estado = 3; // Partiu linha de chegada!
	}

	/*
	// Retirar o Ponto de Apoio já atingido
	for(int i = 0; i < numeroPontos-1; i++){
		vetorPontosApoio[i] = vetorPontosApoio[i+1];
	}
	*/

}

void linhaReta(){
	velocidade = 80;
	changeMotorSpeed(motor_dc, velocidade);
	posicaoAtual = encontrarPosicao(RSSI_1, RSSI_2, RSSI_3);

	// Obtém posição do Magnetômetro
	getAngulo(hi2c2, PontosMagnetoXY);
	if((abs(PontosMagnetoXY.x) >= abs(posicaoAtual.x)) || (abs(PontosMagnetoXY.x) >= abs(posicaoAtual.x))){
		estado = 1;
		n++;
	}
}

void retaFinal(){
	// Considerando que o barco deve passar pelo portal
	posicaoAtual = encontrarPosicao(RSSI_1, RSSI_2, RSSI_3);

	// Teorema de Pitágoras - Determinar a distância em linha reta entre dois pontos
	d = sqrt(pow((posicaoAtual.x - B1_chegada.x),2)+ pow((posicaoAtual.y - B1_chegada.y),2));

	// Reduz a velocidade do motor conforme a distância entre a linha de chegada vai diminuindo
	if((d > 3) || (d <= 5)){
		velocidade = 30;
	}
	if((d > 2) || (d <= 2)){
		velocidade = 20;
	}
	if((d > 0) || (d <= 1)){
		velocidade = 10;
	}
	if(d == 0){
		velocidade = 0; // Chegamos!!! E relâmpago cruza pelo portal!!!
	}

	changeMotorSpeed(motor_dc, velocidade);

	if((angulo > 0) || (angulo <= 5)){
		A = (B1_chegada.x - posicaoAtual.x);
		C = (B1_chegada.y - posicaoAtual.y);
		angulo = (uint16_t) ((180/3.14)*atan2(A,C));

		// O ângulo que deve ser passado ao servo motor é o contrário a esse
		angulo = -1 * angulo;

		// envia ângulo ao Servo
		setPWMAngulo(htim3, TIM_CHANNEL_2, 12500, angulo+offset);
	}
}
