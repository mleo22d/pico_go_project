#include "pico/stdlib.h"
#include "display/st7789.h"

int main() {
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(100);
    }

    sleep_ms(1000);
    
    st7789_init();
    st7789_fill(0xFFFF);  // blanco

    while (1) {
        sleep_ms(500);
    }
}
