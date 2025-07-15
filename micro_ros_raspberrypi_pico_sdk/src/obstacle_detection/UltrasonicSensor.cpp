#include "UltrasonicSensor.h"

UltrasonicSensor::UltrasonicSensor() {
    gpio_init(_trig);
    gpio_set_dir(_trig, GPIO_OUT);
    gpio_put(_trig, 0);

    gpio_init(_echo);
    gpio_set_dir(_echo, GPIO_IN);
}

float UltrasonicSensor::read_distance_cm() {
    gpio_put(_trig, 0);
    sleep_us(2);
    gpio_put(_trig, 1);
    sleep_us(10);
    gpio_put(_trig, 0);

    absolute_time_t start = get_absolute_time();
    while (gpio_get(_echo) == 0) {
        if (absolute_time_diff_us(start, get_absolute_time()) > 30000) return -1;
    }
    absolute_time_t signal_start = get_absolute_time();

    while (gpio_get(_echo) == 1) {
        if (absolute_time_diff_us(signal_start, get_absolute_time()) > 30000) return -1;
    }
    absolute_time_t signal_end = get_absolute_time();

    int64_t duration = absolute_time_diff_us(signal_start, signal_end);
    return duration * 0.0343f / 2.0f;
}
