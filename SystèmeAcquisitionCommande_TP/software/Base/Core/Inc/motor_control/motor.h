/*
 * motor.h
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#ifndef INC_MOTOR_CONTROL_MOTOR_H_
#define INC_MOTOR_CONTROL_MOTOR_H_

#include "user_interface/shell.h"
#include "stm32g4xx_hal.h"
#include "tim.h"

#define MOTOR_MAX_SPEED 100  // 100% de la vitesse maximale

int motor_init(void);
int motor_set_speed(uint16_t speed);
int motor_start(void);
int motor_stop(void);
uint16_t motor_get_speed(void);
int motor_ramp_to_speed(uint16_t target_speed, uint16_t step, uint16_t period_ms);
void motor_ramp_task(void);


#endif /* INC_MOTOR_CONTROL_MOTOR_H_ */
