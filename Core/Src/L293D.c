/*
 * L293D.c
 *
 */

#include <L293D.h>


void configMotor(motor_dc* motor, TIM_HandleTypeDef timer, uint32_t timerChannel)
{
	motor->timer = timer;
	motor->timerChannel = timerChannel;
	motor->configured = 1;
}

void configHC595(HC595* hc595, GPIO_TypeDef* SRCLK_port, uint16_t SRCLK_pin, GPIO_TypeDef* RCLK_port, uint16_t RCLK_pin, GPIO_TypeDef* SER_DATA_port, uint16_t SER_DATA_pin)
{
	hc595->SRCLK.port = SRCLK_port;
	hc595->SRCLK.pin = SRCLK_pin;
	hc595->RCLK.port = RCLK_port;
	hc595->RCLK.pin = RCLK_pin;
	hc595->SER_DATA.port = SER_DATA_port;
	hc595->SER_DATA.pin = SER_DATA_pin;
	hc595->currentConfiguration = 0;
	hc595->configured = 1;
}

void changeMotorOrientation(HC595* hc595, uint8_t orientation)
{

	uint16_t clock_timeout = 10000;
	uint16_t latch_timeout = 1000;

	// Seta o clock para low
	HAL_GPIO_WritePin(hc595->RCLK.port, hc595->RCLK.pin, 0);

	for(int8_t i = 7; i >= 0; --i)
	{
		uint8_t bit = orientation & (0x1 << i);
		HAL_GPIO_WritePin(hc595->SER_DATA.port, hc595->SER_DATA.pin, bit);
		HAL_GPIO_WritePin(hc595->RCLK.port, hc595->RCLK.pin, 1);
		while(clock_timeout--);
		HAL_GPIO_WritePin(hc595->RCLK.port, hc595->RCLK.pin, 0);
	}
	// Inicia a transmissão
	HAL_GPIO_WritePin(hc595->SRCLK.port, hc595->SRCLK.pin, 1);
	while(latch_timeout--);
	HAL_GPIO_WritePin(hc595->SRCLK.port, hc595->SRCLK.pin, 0);
}


void changeMotorSpeed(motor_dc* motor_dc, uint8_t speed)
{
	//motor_dc->timer
	uint8_t motor_speed = 0;
	if(speed >= 100)
	{
		motor_speed = 100;
	}
	else
	{
		motor_speed = speed;
	}

	switch(motor_dc->timerChannel)
	{
		// Canal 1 do timer
		case(1):
			motor_dc->timer.Instance->CCR1 = motor_dc->timer.Instance->ARR * motor_speed / 100;
			break;
		// Canal 2 do timer
		case(4):
		    motor_dc->timer.Instance->CCR2 = motor_dc->timer.Instance->ARR * motor_speed / 100;
			break;
		// Canal 3 do timer
		case(8):
			motor_dc->timer.Instance->CCR3 = motor_dc->timer.Instance->ARR * motor_speed / 100;
			break;
		// Canal 4 do timer
		case(16):
			motor_dc->timer.Instance->CCR4 = motor_dc->timer.Instance->ARR * motor_speed / 100;
			break;
	}
}
