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

void reset(UART_HandleTypeDef huart);
void start(UART_HandleTypeDef huart);
void getResponse(UART_HandleTypeDef huart, char * resposta);
void programarBluetooth(UART_HandleTypeDef huart);
void localizarBeacons(UART_HandleTypeDef huart, char * resposta);
#endif /* INC_BLUETOOTH_H_ */
