#ifndef SSD1306_H
#define SSD1306_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hardware/i2c.h"

#define SSD1306_WIDTH   128
#define SSD1306_HEIGHT  64
#define SSD1306_PAGES   (SSD1306_HEIGHT / 8)

typedef struct {
    i2c_inst_t *i2c;
    uint8_t addr;
    uint8_t col_offset; // 0 for SSD1306, 2 for SH1106
    uint8_t buffer[SSD1306_PAGES][SSD1306_WIDTH];
} ssd1306_t;

void ssd1306_init(ssd1306_t *d, i2c_inst_t *i2c, uint8_t addr, uint8_t col_offset);
void ssd1306_clear(ssd1306_t *d);
void ssd1306_flush(ssd1306_t *d);
void ssd1306_set_pixel(ssd1306_t *d, int x, int y, bool on);
void ssd1306_draw_char(ssd1306_t *d, int x, int y, char c, uint8_t scale);
void ssd1306_draw_text(ssd1306_t *d, int x, int y, const char *s, uint8_t scale);

#endif
