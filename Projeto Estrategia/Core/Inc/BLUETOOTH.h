/*
 * bluetooth.h
 *
 *  Created on: Nov 4, 2023
 *      Author: jac-l
 */

#ifndef INC_BLUETOOTH_H_
#define INC_BLUETOOTH_H_

#include "string.h"
#include "stm32f4xx_hal.h"
#include "stdio.h"
#include "stdlib.h"

void reset(UART_HandleTypeDef huart);
void start(UART_HandleTypeDef huart);
void getResponse(UART_HandleTypeDef huart, char * resposta);
void programarBluetooth(UART_HandleTypeDef huart);
void localizarBeacons(UART_HandleTypeDef huart, char * resposta);
void processarInformacoesBluetooth(const char* informacoes, char* numerosEncontrados);
//void processarInformacoesBluetooth(char* numerosEncontrados);
#endif /* INC_BLUETOOTH_H_ */
