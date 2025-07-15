#ifndef _ST7789_H_
#define _ST7789_H_

#include "hardware/spi.h"

#ifdef __cplusplus
extern "C" {
#endif

void st7789_init();
void st7789_set_cursor(uint16_t x, uint16_t y);
void st7789_put(uint16_t color);
void st7789_fill(uint16_t color);

#ifdef __cplusplus
}
#endif

#endif
