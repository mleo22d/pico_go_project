#include "pico/stdlib.h"
#include "hardware/pwm.h"

const uint SERVO_PIN = 0;
#define LED1_PIN 6
#define LED2_PIN 7
#define LED3_PIN 14
#define LED4_PIN 15

void init_leds() {
    gpio_init(LED1_PIN);
    gpio_set_dir(LED1_PIN, GPIO_OUT);
    gpio_put(LED1_PIN, 1);

    gpio_init(LED2_PIN);
    gpio_set_dir(LED2_PIN, GPIO_OUT);
    gpio_put(LED2_PIN, 1);

    gpio_init(LED3_PIN);
    gpio_set_dir(LED3_PIN, GPIO_OUT);
    gpio_put(LED3_PIN, 1);

    gpio_init(LED4_PIN);
    gpio_set_dir(LED4_PIN, GPIO_OUT);
    gpio_put(LED4_PIN, 1);
}

void servo_init(uint pin) {
    gpio_init(SERVO_PIN);
    gpio_set_dir(SERVO_PIN, true);  // salida
    gpio_put(SERVO_PIN, true);      // mantener HIGH mientras arranca
    gpio_set_function(pin, GPIO_FUNC_PWM);
    uint slice = pwm_gpio_to_slice_num(pin);
    pwm_set_clkdiv(slice, 64.0);        // Divide el reloj (más resolución)
    pwm_set_wrap(slice, 39062);         // Periodo de 20 ms (50 Hz)
    pwm_set_enabled(slice, true);
}

void servo_write(uint pin, int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    float pulse_ms = 1.0 + (angle / 180.0f) * 1.0f;  // 1.0 ms a 2.0 ms
    uint level = (pulse_ms / 20.0f) * 39062;
    pwm_set_gpio_level(pin, level);
}

int main() {
    stdio_init_all();
    servo_init(SERVO_PIN);
    // Forzar posición inicial estable
    for (int i = 0; i < 20; i++) {
        servo_write(SERVO_PIN, 150);  // tu “cero invertido”
        sleep_ms(20);
    }
      
    init_leds();

    while (true) {

        //servo_write(SERVO_PIN, 180);
        
        // Subida de 0° a 45° suave
        for (int angle = 0; angle <= 60; angle += 1) {
            servo_write(SERVO_PIN, 180-angle);
            sleep_ms(5);  // Ajusta para suavidad/velocidad
        }

        sleep_ms(2000);  // Espera 2 s
        
        // Bajada de 45° a 0° suave
        for (int angle = 60; angle >= 0; angle -= 1) {
            servo_write(SERVO_PIN, 180-angle);
            sleep_ms(5);
        }

        sleep_ms(1000);  // Espera 2 s
        
       
    }
}