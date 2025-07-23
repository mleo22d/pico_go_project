#include "Motor.h"
#include <cstdio>

// Constructor vacío
Motor::Motor() {
    gpio_init(_ain1); gpio_set_dir(_ain1, GPIO_OUT);
    gpio_init(_ain2); gpio_set_dir(_ain2, GPIO_OUT);
    gpio_init(_bin1); gpio_set_dir(_bin1, GPIO_OUT);
    gpio_init(_bin2); gpio_set_dir(_bin2, GPIO_OUT);

    gpio_set_function(_pwma, GPIO_FUNC_PWM);
    gpio_set_function(_pwmb, GPIO_FUNC_PWM);

    uint slice_a = pwm_gpio_to_slice_num(_pwma);
    uint slice_b = pwm_gpio_to_slice_num(_pwmb);
    pwm_set_wrap(slice_a, 65535);
    pwm_set_wrap(slice_b, 65535);
    pwm_set_enabled(slice_a, true);
    pwm_set_enabled(slice_b, true);
}

// The function recieves a number between 0 to 100. Where 100 is the max speed and 0 stop
void Motor::set_speed(uint pin, int speed_percent) {

    if (speed_percent < 0 || speed_percent > 100) printf("\nFuera de rango %d", speed_percent);
    if (speed_percent < 0) speed_percent = 0; 
    if (speed_percent > 100) speed_percent = 100;
    
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), speed_percent * 65535 / 100);
}

void Motor::forward(int speed) {
    gpio_put(_ain1, 0); gpio_put(_ain2, 1);
    gpio_put(_bin1, 0); gpio_put(_bin2, 1);
    set_speed(_pwma, speed);
    set_speed(_pwmb, speed);
}

void Motor::stop() {
    gpio_put(_ain1, 0); gpio_put(_ain2, 0);
    gpio_put(_bin1, 0); gpio_put(_bin2, 0);
    set_speed(_pwma, 0);
    set_speed(_pwmb, 0);
}

void Motor::reverse(int speed) {
    gpio_put(_ain1, 1); gpio_put(_ain2, 0);
    gpio_put(_bin1, 1); gpio_put(_bin2, 0);
    set_speed(_pwma, speed);
    set_speed(_pwmb, speed);
}

void Motor::right(int speed) {
    gpio_put(_ain1, 0); gpio_put(_ain2, 1);
    gpio_put(_bin1, 1); gpio_put(_bin2, 0);
    set_speed(_pwma, speed);
    set_speed(_pwmb, int(speed/2.0));
}

void Motor::left(int speed) {
    gpio_put(_ain1, 0); gpio_put(_ain2, 0);
    gpio_put(_bin1, 0); gpio_put(_bin2, 1);
    set_speed(_pwma, 0);
    set_speed(_pwmb, speed);
}

void Motor::set_motor(int left, int right) {
    if ((left >= 0) and (left <= 100)) {
        gpio_put(_ain1, 0);
        gpio_put(_ain2, 1);
        set_speed(_pwma, left);
    }
    else if ((left < 0) and (left >= -100)) {
        gpio_put(_ain1, 1);
        gpio_put(_ain2, 0);
        set_speed(_pwma, -left);;
    }

    if ((right >= 0) and (right <= 100)) {
        gpio_put(_bin1, 0);
        gpio_put(_bin2, 1);
        set_speed(_pwmb, right);;
    }
    else if ((right < 0) and (right >= -100)) {
        gpio_put(_bin1, 1);
        gpio_put(_bin2, 0);
        set_speed(_pwmb, -right);
    }
}
