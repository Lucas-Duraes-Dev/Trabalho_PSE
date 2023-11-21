/*
 * SISTEMA_NAVEGACAO.h
 *
 *  Created on: Nov 19, 2023
 *      Author: Stephanie
 */

#ifndef INC_SISTEMA_NAVEGACAO_H_
#define INC_SISTEMA_NAVEGACAO_H_

#include "MAGNETOMETRO.h"
#include "SERVOMOTOR.h"
#include "L293D.h"
#include "BLUETOOTH.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

#define offset 15

// Sinais de controle
uint8_t anguloDirecao; // intervalo de -30º a 30º (considerando o offset de 15º)
uint8_t aceleracaoMotor; // intervalo de 0 a 100%

// Coordenadas das chegadas intermediárias
/* A estratégia utilizada consistem em criar pontos de apoio
 * ao longo do lago, formando uma hipérbole que representará o
 * caminho que o barco deve percorrer até que chegue ao destino
 * final. Com esse método, a ideia é que com a definição do trajeto,
 * o barco tenha mais informações sobre a rota e cometa menos desvios.
 *
 * Como o lago tem um formato aproximado de um triângulo cujas dimensões
 * são 29,80m (comprimento) x 19,90m (largura) x 31,35m (hipotenusa)
 * Cada ponto distante entre si aproximadamente 8m.
 */

Coordenadas B1_chegada = {-19.866733, -43.964666}; // Chegada

Coordenadas vetorPontosApoio[] = {{-19.866482, -43.964610},
							{-19.866517, -43.964638},
							{-19.866554, -43.964654},
							{-19.866597, -43.964671},
							{-19.866626, -43.964679},
							{-19.866646, -43.964683},
							{-19.866689, -43.964671}};


Coordenadas posicaoAtual;
Coordenadas PontosMagnetoXY;
uint8_t numeroPontos = 7;
uint16_t estado;
uint16_t angulo;
uint16_t velocidade;
int n; // contador
float d, A, C;


// FUNÇÕES
Coordenadas encontrarPosicao(float RSSI_1, float RSSI_2, float RSSI_3);
void pilotoAutomatico();
void estadoInicial();
void proximoDestino();
void linhaReta();
void retaFinal();


#endif /* INC_SISTEMA_NAVEGACAO_H_ */
