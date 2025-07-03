#ifndef MOTOR_H
#define MOTOR_H

#include "hardware/pwm.h"
#include "pico/stdlib.h"


// Pin Definitions
#define PWMA 16
#define AIN1 18
#define AIN2 17
#define PWMB 21
#define BIN1 19
#define BIN2 20

// Functions
void motor_setup();
void forward(float speed);
void stop();
void left(float speed);
void right(float speed);
void reverse(float speed);

#endif