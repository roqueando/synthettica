#include "ssd1306.h"

#include <string.h>

static void write_cmd(ssd1306_t *d, uint8_t cmd) {
    uint8_t buf[2] = {0x00, cmd};
    i2c_write_blocking(d->i2c, d->addr, buf, 2, false);
}

static void write_cmd_arg(ssd1306_t *d, uint8_t cmd, uint8_t arg) {
    uint8_t buf[3] = {0x00, cmd, arg};
    i2c_write_blocking(d->i2c, d->addr, buf, 3, false);
}

void ssd1306_init(ssd1306_t *d, i2c_inst_t *i2c, uint8_t addr, uint8_t col_offset) {
    d->i2c = i2c;
    d->addr = addr;
    d->col_offset = col_offset;

    write_cmd(d, 0xAE);                 // display off
    write_cmd_arg(d, 0x20, 0x02);       // memory addressing mode: page
    write_cmd(d, 0x40);                 // display start line = 0
    write_cmd(d, 0xA1);                 // segment remap
    write_cmd(d, 0xC8);                 // COM scan direction (remapped)
    write_cmd_arg(d, 0xA8, SSD1306_HEIGHT - 1); // multiplex ratio
    write_cmd_arg(d, 0xD3, 0x00);       // display offset = 0
    write_cmd_arg(d, 0xDA, 0x12);       // COM pins hardware config (128x64)
    write_cmd_arg(d, 0x81, 0xCF);       // contrast
    write_cmd_arg(d, 0xD9, 0xF1);       // pre-charge period
    write_cmd_arg(d, 0xDB, 0x40);       // VCOMH deselect level
    write_cmd(d, 0xA4);                 // output follows RAM
    write_cmd(d, 0xA6);                 // normal (not inverted)
    write_cmd_arg(d, 0x8D, 0x14);       // charge pump enable
    write_cmd(d, 0xAF);                 // display on

    ssd1306_clear(d);
    ssd1306_flush(d);
}

void ssd1306_clear(ssd1306_t *d) {
    memset(d->buffer, 0, sizeof(d->buffer));
}

void ssd1306_flush(ssd1306_t *d) {
    for (uint8_t page = 0; page < SSD1306_PAGES; page++) {
        write_cmd(d, 0xB0 | page);
        write_cmd(d, 0x00 + (d->col_offset & 0x0F));
        write_cmd(d, 0x10 + ((d->col_offset >> 4) & 0x0F));

        uint8_t buf[SSD1306_WIDTH + 1];
        buf[0] = 0x40; // data
        memcpy(buf + 1, d->buffer[page], SSD1306_WIDTH);
        i2c_write_blocking(d->i2c, d->addr, buf, sizeof(buf), false);
    }
}

void ssd1306_set_pixel(ssd1306_t *d, int x, int y, bool on) {
    if (x < 0 || x >= SSD1306_WIDTH || y < 0 || y >= SSD1306_HEIGHT) {
        return;
    }
    uint8_t page = (uint8_t)(y >> 3);
    uint8_t bit = (uint8_t)(1u << (y & 7));
    if (on) {
        d->buffer[page][x] |= bit;
    } else {
        d->buffer[page][x] &= (uint8_t)~bit;
    }
}

// 5x7 font, one byte per column, bit 0 = top row.
static const uint8_t font5x7[23][5] = {
    {0x00, 0x00, 0x00, 0x00, 0x00}, // ' '  (0)
    {0x3E, 0x51, 0x49, 0x45, 0x3E}, // '0'  (1)
    {0x00, 0x42, 0x7F, 0x40, 0x00}, // '1'
    {0x42, 0x61, 0x51, 0x49, 0x46}, // '2'
    {0x21, 0x41, 0x45, 0x4B, 0x31}, // '3'
    {0x18, 0x14, 0x12, 0x7F, 0x10}, // '4'
    {0x27, 0x45, 0x45, 0x45, 0x39}, // '5'
    {0x3C, 0x4A, 0x49, 0x49, 0x30}, // '6'
    {0x01, 0x71, 0x09, 0x05, 0x03}, // '7'
    {0x36, 0x49, 0x49, 0x49, 0x36}, // '8'
    {0x06, 0x49, 0x49, 0x29, 0x1E}, // '9'
    {0x7F, 0x09, 0x09, 0x09, 0x01}, // 'F'  (11)
    {0x7F, 0x09, 0x19, 0x29, 0x46}, // 'R'
    {0x7F, 0x49, 0x49, 0x49, 0x41}, // 'E'
    {0x3E, 0x41, 0x51, 0x21, 0x5E}, // 'Q'
    {0x7F, 0x08, 0x08, 0x08, 0x7F}, // 'H'
    {0x61, 0x51, 0x49, 0x45, 0x43}, // 'z'
    {0x26, 0x49, 0x49, 0x49, 0x32}, // 'S'
    {0x00, 0x41, 0x7F, 0x41, 0x00}, // 'I'
    {0x7F, 0x04, 0x08, 0x10, 0x7F}, // 'N'
    {0x01, 0x01, 0x7F, 0x01, 0x01}, // 'T'  (20)
    {0x7E, 0x11, 0x11, 0x11, 0x7E}, // 'A'
    {0x3F, 0x40, 0x38, 0x40, 0x3F}, // 'W'
};

static int font_index(char c) {
    if (c == ' ') return 0;
    if (c >= '0' && c <= '9') return 1 + (c - '0');
    switch (c) {
        case 'F': return 11;
        case 'R': return 12;
        case 'E': return 13;
        case 'Q': return 14;
        case 'H': return 15;
        case 'z': return 16;
        case 'S': return 17;
        case 'I': return 18;
        case 'N': return 19;
        case 'T': return 20;
        case 'A': return 21;
        case 'W': return 22;
        default: return 0;
    }
}

void ssd1306_draw_char(ssd1306_t *d, int x, int y, char c, uint8_t scale) {
    const uint8_t *g = font5x7[font_index(c)];
    for (int col = 0; col < 5; col++) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; row++) {
            if (bits & (1u << row)) {
                for (uint8_t sy = 0; sy < scale; sy++) {
                    for (uint8_t sx = 0; sx < scale; sx++) {
                        ssd1306_set_pixel(d, x + col * scale + sx, y + row * scale + sy, true);
                    }
                }
            }
        }
    }
}

void ssd1306_draw_text(ssd1306_t *d, int x, int y, const char *s, uint8_t scale) {
    int cx = x;
    while (*s) {
        ssd1306_draw_char(d, cx, y, *s, scale);
        cx += 6 * scale; // 5px glyph + 1px spacing
        s++;
    }
}
