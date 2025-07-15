#ifndef ULTRASONIC_SENSOR_H
#define ULTRASONIC_SENSOR_H

#include "pico/stdlib.h"

class UltrasonicSensor {
public:
    UltrasonicSensor();
    float read_distance_cm();

private:
    const uint _trig = 14;    
    const uint _echo = 15;
};

#endif
