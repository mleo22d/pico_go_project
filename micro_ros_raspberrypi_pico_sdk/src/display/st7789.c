#include "st7789.h"
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#define LCD_WIDTH  240
#define LCD_HEIGHT 280

#define PIN_SCK  10
#define PIN_MOSI 11
#define PIN_DC   8
#define PIN_RST  12
#define PIN_BL   13

#define SPI_PORT spi1

static void st7789_cmd(uint8_t cmd) {
    gpio_put(PIN_DC, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
}

static void st7789_data(const uint8_t *data, size_t len) {
    gpio_put(PIN_DC, 1);
    spi_write_blocking(SPI_PORT, data, len);
}

void st7789_set_cursor(uint16_t x, uint16_t y) {
    uint8_t col[] = { x >> 8, x & 0xFF, x >> 8, x & 0xFF };
    uint8_t row[] = { y >> 8, y & 0xFF, y >> 8, y & 0xFF };

    st7789_cmd(0x2A); st7789_data(col, 4);
    st7789_cmd(0x2B); st7789_data(row, 4);
    st7789_cmd(0x2C); // RAMWR
    spi_set_format(SPI_PORT, 16, 0, 0, SPI_MSB_FIRST);
}

void st7789_put(uint16_t color) {
    printf("Sending pixel 0x%04X\n", color);
    uint8_t data[] = { color >> 8, color & 0xFF };
    gpio_put(PIN_DC, 1);
    spi_write_blocking(SPI_PORT, data, 2);
}

void st7789_fill(uint16_t color) {
    uint8_t pixel[] = { color >> 8, color & 0xFF };
    st7789_cmd(0x2A); st7789_data((uint8_t[]){ 0x00, 0, (LCD_WIDTH-1) >> 8, (LCD_WIDTH-1) & 0xFF }, 4);
    st7789_cmd(0x2B); st7789_data((uint8_t[]){ 0x00, 0, (LCD_HEIGHT-1) >> 8, (LCD_HEIGHT-1) & 0xFF }, 4);
    printf("Calling RAMWR\n");
    st7789_cmd(0x2C);

    gpio_put(PIN_DC, 1);
    printf("DC set to 1 (data mode)\n");
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; i++) {
        spi_write_blocking(SPI_PORT, pixel, 2);
    }
}

void st7789_init() {
    gpio_init(PIN_DC);  gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_init(PIN_RST); gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_init(PIN_BL);  gpio_set_dir(PIN_BL, GPIO_OUT);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    spi_init(SPI_PORT, 6 * 1000 * 1000);
    spi_set_format(SPI_PORT, 8, 0, 0, SPI_MSB_FIRST);

    gpio_put(PIN_RST, 0); sleep_ms(50);
    gpio_put(PIN_RST, 1); sleep_ms(50);

    const struct {
        uint8_t cmd;
        uint8_t data[16];
        uint8_t datalen;
        uint8_t delay_ms;
    } init_seq[] = {
        {0x01, {}, 0, 150}, // SWRESET
        {0x11, {}, 0, 255}, // SLPOUT
        {0x3A, {0x55}, 1, 10}, // COLMOD: 16-bit
        {0x36, {0x00}, 1, 10}, // MADCTL
        {0x21, {}, 0, 10}, // INVON
        {0x13, {}, 0, 10}, // NORON
        {0x29, {}, 0, 100}, // DISPON
    };

    for (int i = 0; i < sizeof(init_seq)/sizeof(*init_seq); i++) {
        st7789_cmd(init_seq[i].cmd);
        if (init_seq[i].datalen) {
            st7789_data(init_seq[i].data, init_seq[i].datalen);
        }
        if (init_seq[i].delay_ms) {
            sleep_ms(init_seq[i].delay_ms);
        }
    }

    gpio_put(PIN_BL, 1);  // Backlight on
}
