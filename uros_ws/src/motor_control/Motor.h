#ifndef MOTOR_H
#define MOTOR_H

#include "hardware/pwm.h"
#include "pico/stdlib.h"

class Motor {
public:
    Motor();
    void forward(int speed);
    void stop();
    void left(int speed);
    void right(int speed);
    void reverse(int speed);
    void drop_right(int speed);
    void drop_left(int speed);
    void set_motor(int left, int right);
    void set_speed(uint pin, int speed_percent);
    
    
private:
    const uint _ain1 = 18;
    const uint _ain2 = 17;
    const uint _pwma = 16;
    const uint _bin1 = 19;
    const uint _bin2 = 20;
    const uint _pwmb = 21;
};



#endif