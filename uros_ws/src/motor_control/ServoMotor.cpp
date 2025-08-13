#include "ServoMotor.h"
#include "hardware/gpio.h"
#include "servo.pio.h"

ServoMotor::ServoMotor() {
    _sm = pio_claim_unused_sm(_pio, true);
    uint offset = pio_add_program(_pio, &servo_program);

    pio_gpio_init(_pio, _pin);
    pio_sm_set_consecutive_pindirs(_pio, _sm, _pin, 1, true);

    pio_sm_config c = servo_program_get_default_config(offset);
    sm_config_set_set_pins(&c, _pin, 1);
    sm_config_set_clkdiv(&c, 125.0f); // 1 MHz

    pio_sm_init(_pio, _sm, offset, &c);
    pio_sm_set_enabled(_pio, _sm, true);
}

void ServoMotor::write_us(uint16_t high_us) {
    if (high_us < 1000) high_us = 1000;
    if (high_us > 2000) high_us = 2000;
    uint32_t low_us = 20000u - high_us;
    pio_sm_put_blocking(_pio, _sm, high_us);
    pio_sm_put_blocking(_pio, _sm, low_us);
};

void ServoMotor::write_deg(int deg) {
    if (deg < 0) deg = 0;
    if (deg > 180) deg = 180;
    // Mapeo lineal 0..180 -> min_us..max_us
    uint16_t us = (uint16_t)(1000 + (1000* deg ) / 180);
    write_us(us);
}