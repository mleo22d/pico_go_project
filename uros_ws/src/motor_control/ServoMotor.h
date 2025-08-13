#pragma once
#include <cstdint>
#include "hardware/pio.h"
#include "pico/time.h"

class ServoMotor {
public:
    ServoMotor();
    void write_us(uint16_t high_us);
    void write_deg(int deg);
private:
    PIO _pio = pio0;
    uint _pin = 0;
    uint _sm;
};
