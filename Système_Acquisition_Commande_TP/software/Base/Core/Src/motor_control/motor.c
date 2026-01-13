/*
 * motor.c
 *
 *  Created on: Nov 11, 2025
 *      Author: nicolas
 */

#include "motor_control/motor.h"
#include "user_interface/shell.h"

static uint16_t motor_current_speed = 50;
static uint8_t  motor_running       = 0;

static uint16_t motor_target_speed   = 50;
static uint16_t motor_ramp_step      = 1;
static uint16_t motor_ramp_period_ms = 100;
static uint8_t  motor_ramp_active    = 0;
static uint32_t motor_next_ramp_tick = 0;

int motor_init(){
    motor_set_speed(50);

    motor_target_speed   = 50;
    motor_ramp_step      = 1;
    motor_ramp_period_ms = 100;
    motor_ramp_active    = 0;

    motor_running = 0;
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    int size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Motor init: speed=50%%, PWM stopped\r\n");
    hshell1.drv.transmit(hshell1.print_buffer, size);
    return 0;
}


int motor_start(void)
{
    int size;

    if (motor_running) {
        size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Motor already started\r\n");
        hshell1.drv.transmit(hshell1.print_buffer, size);
        return 0;
    }

    // Rapport cyclique 50% au démarrage (vitesse nulle)
    motor_set_speed(50);

    // PWM CH1 + CH1N
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1) != HAL_OK) {

        size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "ERROR: PWM1 start\r\n");
        hshell1.drv.transmit(hshell1.print_buffer, size);
        return -1;
    }

    // PWM CH2 + CH2N
    if (HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2) != HAL_OK ||
        HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2) != HAL_OK) {

        size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "ERROR: PWM2 start\r\n");
        hshell1.drv.transmit(hshell1.print_buffer, size);
        return -1;
    }

    motor_running = 1;
    size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "Motor started (duty=50%%)\r\n");
    hshell1.drv.transmit(hshell1.print_buffer, size);

    return 0;
}

int motor_stop(void)
{
    int size;

    if (!motor_running) {
        size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Motor already stopped\r\n");
        hshell1.drv.transmit(hshell1.print_buffer, size);
        return 0;
    }

    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2);

    motor_running = 0;

    size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "Motor stopped\r\n");
    hshell1.drv.transmit(hshell1.print_buffer, size);

    return 0;
}


int motor_set_speed(uint16_t speed)
{
    int size;

    // Saturation à la valeur max autorisée
    if (speed > MOTOR_MAX_SPEED) {
        speed = MOTOR_MAX_SPEED;
    }

    motor_current_speed = speed;

    // Conversion speed [0..MOTOR_MAX_SPEED] -> duty-cycle sur ARR
    uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim1); // ex: ARR = 1700 - 1
    uint32_t ccr     = (uint32_t)speed * (arr + 1) / MOTOR_MAX_SPEED;
    uint32_t ccr_inv = (uint32_t)(MOTOR_MAX_SPEED - speed) * (arr + 1) / MOTOR_MAX_SPEED;

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_inv);	// PWM complémentaire

    size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                    "Commande vitesse = %u -> CCR1 = %lu / %lu\r\n",
                    speed, (unsigned long)ccr, (unsigned long)(arr + 1));
    hshell1.drv.transmit(hshell1.print_buffer, size);

    return 0;
}

uint16_t motor_get_speed(void)
{
    return motor_current_speed;
}

int motor_ramp_to_speed(uint16_t target_speed,
                        uint16_t step,
                        uint16_t period_ms)
{
    // Clamp de la cible
    if (target_speed > MOTOR_MAX_SPEED) {
        target_speed = MOTOR_MAX_SPEED;
    }

    if (step == 0) {
        step = 1;
    }

    motor_target_speed   = target_speed;
    motor_ramp_step      = step;
    motor_ramp_period_ms = period_ms;
    motor_ramp_active    = 1;
    motor_next_ramp_tick = HAL_GetTick() + motor_ramp_period_ms;

    int size = snprintf(hshell1.print_buffer, SHELL_PRINT_BUFFER_SIZE,
                        "Ramp config: target=%u step=%u period=%u ms\r\n",
                        motor_target_speed, motor_ramp_step, motor_ramp_period_ms);
    hshell1.drv.transmit(hshell1.print_buffer, size);

    return 0;   // ✅ Retour immédiat, pas de HAL_Delay ici
}


void motor_ramp_task(void)
{
    if (!motor_ramp_active) {
        return;
    }

    uint32_t now = HAL_GetTick();

    // Pas encore le moment de faire un pas
    if ((int32_t)(now - motor_next_ramp_tick) < 0) {
        return;
    }

    motor_next_ramp_tick = now + motor_ramp_period_ms;

    int16_t current = (int16_t)motor_current_speed;
    int16_t target  = (int16_t)motor_target_speed;

    if (current == target) {
        motor_ramp_active = 0;
        return;
    }

    int16_t direction = (target > current) ? 1 : -1;
    current += direction * motor_ramp_step;

    // Ne pas dépasser la cible
    if ((direction > 0 && current > target) ||
        (direction < 0 && current < target)) {
        current = target;
    }

    motor_set_speed((uint16_t)current);

    if (current == target) {
        motor_ramp_active = 0;
    }
}



