#ifndef SERVO_H
#define SERVO_H

#include <stdint.h>

// 爪用サーボ(PC5)。TIM1+DMAによるPWM(50Hz)
#define SERVO_PORT      GPIOC
#define SERVO_PIN       GPIO_PIN_5

#define SERVO_MIN_US    500
#define SERVO_MAX_US    2500

void servo_init(uint16_t pulse_us);
void servo_set_pulse_us(uint16_t pulse_us);

#endif //SERVO_H
