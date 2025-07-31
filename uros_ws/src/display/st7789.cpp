#include "st7789.h"
#include "font.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/time.h"

#include <cstdlib>  // Added for abs() function

#define SPI_PORT spi1
#define PIN_SCK 10
#define PIN_MOSI 11
#define PIN_CS 9
#define PIN_DC 8
#define PIN_RST 12

// ST7789 specific commands
#define ST7789_SWRESET 0x01
#define ST7789_SLPOUT  0x11
#define ST7789_COLMOD  0x3A
#define ST7789_MADCTL  0x36
#define ST7789_CASET   0x2A
#define ST7789_RASET   0x2B
#define ST7789_RAMWR   0x2C
#define ST7789_INVOFF  0x20
#define ST7789_INVON   0x21
#define ST7789_DISPON  0x29


#define X_OFFSET 40
#define Y_OFFSET 53

static void lcd_cmd(uint8_t cmd) {
    gpio_put(PIN_DC, 0);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, &cmd, 1);
    gpio_put(PIN_CS, 1);
}

static void lcd_data(const uint8_t *data, size_t len) {
    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);
    spi_write_blocking(SPI_PORT, data, len);
    gpio_put(PIN_CS, 1);
}

static void lcd_set_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    uint8_t data[4];

    // Apply display offset
    x0 += X_OFFSET;
    x1 += X_OFFSET;
    y0 += Y_OFFSET;
    y1 += Y_OFFSET;

    // Column address set
    lcd_cmd(ST7789_CASET);
    data[0] = (x0 >> 8) & 0xFF;
    data[1] = x0 & 0xFF;
    data[2] = (x1 >> 8) & 0xFF;
    data[3] = x1 & 0xFF;
    lcd_data(data, 4);

    // Row address set  
    lcd_cmd(ST7789_RASET);
    data[0] = (y0 >> 8) & 0xFF;
    data[1] = y0 & 0xFF;
    data[2] = (y1 >> 8) & 0xFF;
    data[3] = y1 & 0xFF;
    lcd_data(data, 4);

    // Write to RAM
    lcd_cmd(ST7789_RAMWR);
}

void st7789_init() {
    // Initialize SPI at 62.5MHz
    spi_init(SPI_PORT, 62500000);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);

    // Initialize control pins
    gpio_init(PIN_CS);
    gpio_init(PIN_DC); 
    gpio_init(PIN_RST);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_set_dir(PIN_DC, GPIO_OUT);
    gpio_set_dir(PIN_RST, GPIO_OUT);

    // Hardware reset sequence
    gpio_put(PIN_CS, 1);
    gpio_put(PIN_RST, 0);
    sleep_ms(10);
    gpio_put(PIN_RST, 1);
    sleep_ms(120);

    // Software reset
    lcd_cmd(ST7789_SWRESET);
    sleep_ms(150);

    // Sleep out
    lcd_cmd(ST7789_SLPOUT);
    sleep_ms(120);

    // Memory Access Control - Fix orientation and mirroring
    lcd_cmd(ST7789_MADCTL);
    // uint8_t madctl_data = 0xA0;  //for oppositie orientation
    uint8_t madctl_data = 0x60;  //for oppositie orientation

    // Try different values if still mirrored:
    // 0x60 = MX+MY, 0xA0 = MX+MV, 0xC0 = MX+MY+MV, etc.
    lcd_data(&madctl_data, 1);

    // Pixel format - 16-bit RGB565
    lcd_cmd(ST7789_COLMOD);
    uint8_t colmod_data = 0x55;  // 16-bit color
    lcd_data(&colmod_data, 1);

    // Inversion off (try INVON if colors look wrong)
    lcd_cmd(ST7789_INVOFF);

    // Display on
    lcd_cmd(ST7789_DISPON);
    sleep_ms(120);

    // Clear screen
    st7789_fill(0x0000);
}

void st7789_fill(uint16_t color) {
    lcd_set_window(0, 0, LCD_WIDTH - 1, LCD_HEIGHT - 1);
    
    // Convert to big-endian for ST7789
    uint8_t data[2] = {(color >> 8) & 0xFF, color & 0xFF};

    gpio_put(PIN_DC, 1);
    gpio_put(PIN_CS, 0);
    
    for (int i = 0; i < LCD_WIDTH * LCD_HEIGHT; ++i) {
        spi_write_blocking(SPI_PORT, data, 2);
    }
    
    gpio_put(PIN_CS, 1);
}

void draw_pixel(int16_t x, int16_t y, uint16_t color) {
    if (x < 0 || x >= LCD_WIDTH || y < 0 || y >= LCD_HEIGHT) return;

    lcd_set_window(x, y, x, y);
    uint8_t data[] = {(color >> 8) & 0xFF, color & 0xFF};
    lcd_data(data, 2);
}

void draw_char(int x, int y, char c, uint16_t color) {
    if (c < 0 || c > 127) c = 0;

    for (int row = 0; row < 8; ++row) {
        uint8_t row_data = font8x8_basic[(int)c][row];
        for (int col = 0; col < 8; ++col) {
            if (row_data & (1 << col)) {
                draw_pixel(x + col, y + row, color);
            }
        }
    }
}

void draw_text(int x, int y, const char *text, uint16_t color) {
    while (*text) {
        draw_char(x, y, *text++, color);
        x += 8;
    }
}

// Creative additions:

void draw_rect(int x, int y, int width, int height, uint16_t color) {
    for (int i = x; i < x + width && i < LCD_WIDTH; i++) {
        for (int j = y; j < y + height && j < LCD_HEIGHT; j++) {
            draw_pixel(i, j, color);
        }
    }
}

void draw_circle(int cx, int cy, int radius, uint16_t color) {
    for (int x = -radius; x <= radius; x++) {
        for (int y = -radius; y <= radius; y++) {
            if (x*x + y*y <= radius*radius) {
                draw_pixel(cx + x, cy + y, color);
            }
        }
    }
}

void draw_line(int x0, int y0, int x1, int y1, uint16_t color) {
    int dx = abs(x1 - x0);
    int dy = abs(y1 - y0);
    int sx = (x0 < x1) ? 1 : -1;
    int sy = (y0 < y1) ? 1 : -1;
    int err = dx - dy;

    while (1) {
        draw_pixel(x0, y0, color);
        
        if (x0 == x1 && y0 == y1) break;
        
        int e2 = 2 * err;
        if (e2 > -dy) {
            err -= dy;
            x0 += sx;
        }
        if (e2 < dx) {
            err += dx;
            y0 += sy;
        }
    }
}

// Rainbow effect for text
void draw_text_rainbow(int x, int y, const char *text) {
    uint16_t colors[] = {
        0xF800, // Red
        0xFD20, // Orange  
        0xFFE0, // Yellow
        0x07E0, // Green
        0x001F, // Blue
        0x781F, // Purple
    };
    
    int color_idx = 0;
    while (*text) {
        draw_char(x, y, *text++, colors[color_idx]);
        x += 8;
        color_idx = (color_idx + 1) % 6;
    }
}

// Animated progress bar
void draw_progress_bar(int x, int y, int width, int height, float progress, uint16_t bg_color, uint16_t fill_color) {
    // Background
    draw_rect(x, y, width, height, bg_color);
    
    // Fill based on progress (0.0 to 1.0)
    int fill_width = (int)(width * progress);
    draw_rect(x, y, fill_width, height, fill_color);
}

// Simple animation frame for a bouncing dot
void draw_bouncing_dot(int *x, int *y, int *dx, int *dy, uint16_t color) {
    static uint16_t last_color = 0x0000; // Black to clear previous
    
    // Clear previous position
    draw_circle(*x, *y, 3, last_color);
    
    // Update position
    *x += *dx;
    *y += *dy;
    
    // Bounce off edges
    if (*x <= 3 || *x >= LCD_WIDTH - 3) *dx = -*dx;
    if (*y <= 3 || *y >= LCD_HEIGHT - 3) *dy = -*dy;
    
    // Draw new position
    draw_circle(*x, *y, 3, color);
    last_color = color;
}

void draw_large_digit(int x, int y, char digit, uint16_t color, int scale) {
    if (digit < '0' || digit > '9') return;
    
    const uint8_t *font_data = font8x8_basic[(int)digit];
    
    for (int row = 0; row < 8; row++) {
        uint8_t row_data = font_data[row];
        int start_col = -1;
        
        for (int col = 0; col < 8; col++) {
            if (row_data & (1 << col)) {
                // Start a new segment if not already in one
                if (start_col == -1) {
                    start_col = col;
                }
            } else {
                // End of segment - draw horizontal line
                if (start_col != -1) {
                    int length = col - start_col;
                    draw_rect(x + start_col * scale, 
                             y + row * scale,
                             length * scale, 
                             scale, 
                             color);
                    start_col = -1;
                }
            }
        }
        
        // Draw any remaining segment at end of row
        if (start_col != -1) {
            int length = 8 - start_col;
            draw_rect(x + start_col * scale, 
                     y + row * scale,
                     length * scale, 
                     scale, 
                     color);
        }
    }
}