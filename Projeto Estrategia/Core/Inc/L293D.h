/*
 * L293D.h
 *
 * Trabalho 1 para a disciplina de Programação de Sistemas Embarcados
 * O objetivo deste trabalho é controlar um motor DC utilizando o shield para a ponte H
 */

#ifndef INC_L293D_H_
#define INC_L293D_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_tim.h"

typedef struct gpio_t{
	int pin;
	void * port;
} gpio_t;

typedef struct HC595{
	gpio_t SRCLK;
	gpio_t RCLK;
	gpio_t SER_DATA;
	uint8_t currentConfiguration;
	uint8_t configured;
} HC595;

typedef struct motor_dc{
	uint8_t configured; // 1 if the motor was configured correctly; 0 otherwise
	TIM_HandleTypeDef timer;
	uint32_t timerChannel;
} motor_dc;

void configMotor(motor_dc* motor, TIM_HandleTypeDef timer, uint32_t timerChannel);

void configHC595(HC595* hc595, GPIO_TypeDef* SRCLK_port, uint16_t SRCLK_pin, GPIO_TypeDef* RCLK_port, uint16_t RCLK_pin, GPIO_TypeDef* SER_DATA_port, uint16_t SER_DATA_pin);

void changeMotorOrientation(HC595* hc595, uint8_t orientation);

void changeMotorSpeed(motor_dc* motor_dc, uint8_t speed); // Sets motor to a percentage of its maximum speed. Only values from 0-100 are accepted

#endif /* INC_L293D_H_ */
