#include "motor.h"
#include <cstdio>

// Velocity Controll PWM
static void set_speed(uint pin, float speed_normalized) {
    printf("Setting speed %.2f → PWM level: %d\n", speed_normalized, (uint16_t)(speed_normalized * 65535));

    if (speed_normalized < 0.0f) speed_normalized = 0.0f;
    if (speed_normalized > 1.0f) speed_normalized = 1.0f;

    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), (uint16_t)(speed_normalized * 65535));
}

void motor_setup() {
    gpio_init(AIN1); gpio_set_dir(AIN1, GPIO_OUT);
    gpio_init(AIN2); gpio_set_dir(AIN2, GPIO_OUT);
    gpio_init(BIN1); gpio_set_dir(BIN1, GPIO_OUT);
    gpio_init(BIN2); gpio_set_dir(BIN2, GPIO_OUT);

    gpio_set_function(PWMA, GPIO_FUNC_PWM);
    gpio_set_function(PWMB, GPIO_FUNC_PWM);

    uint slice_a = pwm_gpio_to_slice_num(PWMA);
    uint slice_b = pwm_gpio_to_slice_num(PWMB);
    pwm_set_wrap(slice_a, 65535);
    pwm_set_wrap(slice_b, 65535);
    pwm_set_enabled(slice_a, true);
    pwm_set_enabled(slice_b, true);
}

void forward(float speed) {
    gpio_put(AIN1, 0); gpio_put(AIN2, 1);
    gpio_put(BIN1, 0); gpio_put(BIN2, 1);
    set_speed(PWMA, speed);
    set_speed(PWMB, speed);
}

void stop() {
    gpio_put(AIN1, 0); gpio_put(AIN2, 0);
    gpio_put(BIN1, 0); gpio_put(BIN2, 0);
    set_speed(PWMA, 0);
    set_speed(PWMB, 0);
}

void reverse(float speed) {
    gpio_put(AIN1, 1); gpio_put(AIN2, 0);
    gpio_put(BIN1, 1); gpio_put(BIN2, 0);
    set_speed(PWMA, speed);
    set_speed(PWMB, speed);
}

void right(float speed) {
    gpio_put(AIN1, 0); gpio_put(AIN2, 1);
    gpio_put(BIN1, 0); gpio_put(BIN2, 0);
    set_speed(PWMA, speed);
    set_speed(PWMB, 0);
}

void left(float speed) {
    gpio_put(AIN1, 0); gpio_put(AIN2, 0);
    gpio_put(BIN1, 0); gpio_put(BIN2, 1);
    set_speed(PWMA, 0);
    set_speed(PWMB, speed);
}
