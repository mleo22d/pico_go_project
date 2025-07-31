    /*#include "pico/stdlib.h"
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
*/

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ws2812.pio.h"
#include <stdio.h>

#define PIN_WS2812 22
#define NUM_LEDS 4
#define BRIGHTNESS 0.8f  // entre 0.0 y 1.0

uint32_t urgb_u32(uint8_t r, uint8_t g, uint8_t b) {
    return ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
}

void ws2812_program_init(PIO pio, uint sm, uint offset, uint pin, float freq) {
    pio_sm_config c = ws2812_program_get_default_config(offset);
    sm_config_set_sideset_pins(&c, pin);
    sm_config_set_out_shift(&c, false, true, 24);
    sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
    sm_config_set_clkdiv(&c, (float)clock_get_hz(clk_sys) / freq);
    pio_sm_set_consecutive_pindirs(pio, sm, pin, 1, true);
    pio_sm_init(pio, sm, offset, &c);
    pio_sm_set_enabled(pio, sm, true);
}
int main() {
    stdio_init_all();
    sleep_ms(1000);
    printf("Iniciando...\n");

    PIO pio = pio0;
    uint sm = 0;
    uint offset = pio_add_program(pio, &ws2812_program);
    printf("Programa PIO cargado en offset %u\n", offset);

    ws2812_program_init(pio, sm, offset, PIN_WS2812, 800000);
    printf("PIO inicializado\n");

    uint8_t r = 255 * BRIGHTNESS;
    uint8_t g = 255 * BRIGHTNESS;
    uint8_t b = 255 * BRIGHTNESS;
    uint32_t color = urgb_u32(255, 0, 0);  // Rojo fuerte

    printf("Color armado: %06X\n", color);

    while (true) {
        for (int i = 0; i < NUM_LEDS; i++) {
            pio_sm_put_blocking(pio, sm, color << 8u);
        }
        printf("Color enviado\n");
        sleep_ms(1000);
    }
}
