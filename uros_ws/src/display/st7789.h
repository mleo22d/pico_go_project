#pragma once
#include <stdint.h>

// Display dimensions - adjust these based on your specific ST7789 module
#define LCD_WIDTH  240
#define LCD_HEIGHT 240

// Color definitions (RGB565 format)
#define BLACK   0x0000
#define WHITE   0xFFFF
#define RED     0xF800
#define GREEN   0x07E0
#define BLUE    0x001F
#define CYAN    0x07FF
#define MAGENTA 0xF81F
#define YELLOW  0xFFE0
#define ORANGE  0xFD20
#define PURPLE  0x781F
#define PINK    0xF81F
#define GRAY    0x8410

// Basic display functions
void st7789_init(void);
void st7789_fill(uint16_t color);
void draw_pixel(int16_t x, int16_t y, uint16_t color);
void draw_char(int x, int y, char c, uint16_t color);
void draw_text(int x, int y, const char *text, uint16_t color);

// Creative/Enhanced functions
void draw_rect(int x, int y, int width, int height, uint16_t color);
void draw_circle(int cx, int cy, int radius, uint16_t color);
void draw_line(int x0, int y0, int x1, int y1, uint16_t color);
void draw_text_rainbow(int x, int y, const char *text);
void draw_progress_bar(int x, int y, int width, int height, float progress, uint16_t bg_color, uint16_t fill_color);
void draw_bouncing_dot(int *x, int *y, int *dx, int *dy, uint16_t color);
void draw_large_digit(int x, int y, char digit, uint16_t color, int scale);