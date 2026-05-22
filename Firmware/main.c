#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/spi.h"

#define IR_SPACE_1 27
#define IR_SPACE_2 29
#define IR_SPACE_3 34
#define IR_SPACE_4 39
#define IR_SPACE_5 44

#define BTN_SPACE_1 17
#define BTN_SPACE_2 18
#define BTN_SPACE_3 19
#define BTN_SPACE_4 20
#define BTN_SPACE_5 21

#define LED1_R 14
#define LED1_G 15
#define LED1_B 16
#define LED2_R 11
#define LED2_G 12
#define LED2_B 13
#define LED3_R 8
#define LED3_G 9
#define LED3_B 10
#define LED4_R 5
#define LED4_G 6
#define LED4_B 7
#define LED5_R 2
#define LED5_G 3
#define LED5_B 4

#define LCD_SPI    spi1
#define LCD_MOSI   47
#define LCD_SCK    46
#define LCD_CS     45
#define LCD_DC     43
#define LCD_RESET  42
#define LCD_LED    41

#define ILI9341_SWRESET 0x01
#define ILI9341_SLPOUT  0x11
#define ILI9341_DISPON  0x29
#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_MADCTL  0x36
#define ILI9341_PIXFMT  0x3A

#define COLOR_BLACK  0x0000
#define COLOR_RED    0xF800
#define COLOR_GREEN  0x07E0
#define COLOR_BLUE   0x001F
#define COLOR_CYAN   0x07FF
#define COLOR_YELLOW 0xFFE0
#define COLOR_WHITE  0xFFFF
#define COLOR_GRAY   0x8410
#define COLOR_ORANGE 0xFC00

#define LCD_WIDTH  240
#define LCD_HEIGHT 320

#define SPACE_NORMAL      0
#define SPACE_HANDICAP    1
#define SPACE_UNAVAILABLE 2

#define HOLD_THRESHOLD_MS 1500
#define DEBOUNCE_MS       50
#define NUM_SPACES        5

const uint8_t sensor_pins[NUM_SPACES] = {IR_SPACE_1, IR_SPACE_2, IR_SPACE_3, IR_SPACE_4, IR_SPACE_5};
const uint8_t button_pins[NUM_SPACES] = {BTN_SPACE_1, BTN_SPACE_2, BTN_SPACE_3, BTN_SPACE_4, BTN_SPACE_5};
const uint8_t button_controls_space[NUM_SPACES] = {0, 1, 2, 3, 4};

const uint8_t led_pins[NUM_SPACES][3] = {
    {LED1_R, LED1_G, LED1_B}, {LED2_R, LED2_G, LED2_B},
    {LED3_R, LED3_G, LED3_B}, {LED4_R, LED4_G, LED4_B},
    {LED5_R, LED5_G, LED5_B}
};

uint8_t space_type[NUM_SPACES]       = {SPACE_HANDICAP, SPACE_NORMAL, SPACE_NORMAL, SPACE_NORMAL, SPACE_UNAVAILABLE};
bool override_active[NUM_SPACES]     = {false};
bool override_occupied[NUM_SPACES]   = {false};
bool btn_was_pressed[NUM_SPACES]     = {false};
uint32_t btn_press_time[NUM_SPACES]  = {0};
bool btn_hold_fired[NUM_SPACES]      = {false};
bool last_state[NUM_SPACES]          = {false};
uint8_t saved_type[NUM_SPACES]       = {SPACE_HANDICAP, SPACE_NORMAL, SPACE_NORMAL, SPACE_NORMAL, SPACE_UNAVAILABLE};

absolute_time_t occupation_start_time[NUM_SPACES];
uint32_t total_occupied_ms[NUM_SPACES] = {0};
uint32_t occupation_count[NUM_SPACES]  = {0};
uint32_t last_print_time = 0;

const uint8_t font8x8[][8] = {
    {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00}, // (space)
    {0x18,0x3C,0x3C,0x18,0x18,0x00,0x18,0x00}, // !
    {0x36,0x36,0x00,0x00,0x00,0x00,0x00,0x00}, // "
    {0x36,0x36,0x7F,0x36,0x7F,0x36,0x36,0x00}, // #
    {0x0C,0x3E,0x03,0x1E,0x30,0x1F,0x0C,0x00}, // $
    {0x00,0x63,0x33,0x18,0x0C,0x66,0x63,0x00}, // %
    {0x1C,0x36,0x1C,0x6E,0x3B,0x33,0x6E,0x00}, // &
    {0x06,0x06,0x03,0x00,0x00,0x00,0x00,0x00}, // '
    {0x18,0x0C,0x06,0x06,0x06,0x0C,0x18,0x00}, // (
    {0x06,0x0C,0x18,0x18,0x18,0x0C,0x06,0x00}, // )
    {0x00,0x66,0x3C,0xFF,0x3C,0x66,0x00,0x00}, // *
    {0x00,0x0C,0x0C,0x3F,0x0C,0x0C,0x00,0x00}, // +
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x06}, // ,
    {0x00,0x00,0x00,0x3F,0x00,0x00,0x00,0x00}, // -
    {0x00,0x00,0x00,0x00,0x00,0x0C,0x0C,0x00}, // .
    {0x60,0x30,0x18,0x0C,0x06,0x03,0x01,0x00}, // /
    {0x3E,0x63,0x73,0x7B,0x6F,0x67,0x3E,0x00}, // 0
    {0x0C,0x0E,0x0C,0x0C,0x0C,0x0C,0x3F,0x00}, // 1
    {0x1E,0x33,0x30,0x1C,0x06,0x33,0x3F,0x00}, // 2
    {0x1E,0x33,0x30,0x1C,0x30,0x33,0x1E,0x00}, // 3
    {0x38,0x3C,0x36,0x33,0x7F,0x30,0x78,0x00}, // 4
    {0x3F,0x03,0x1F,0x30,0x30,0x33,0x1E,0x00}, // 5
    {0x1C,0x06,0x03,0x1F,0x33,0x33,0x1E,0x00}, // 6
    {0x3F,0x33,0x30,0x18,0x0C,0x0C,0x0C,0x00}, // 7
    {0x1E,0x33,0x33,0x1E,0x33,0x33,0x1E,0x00}, // 8
    {0x1E,0x33,0x33,0x3E,0x30,0x18,0x0E,0x00}, // 9
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x00}, // :
    {0x00,0x0C,0x0C,0x00,0x00,0x0C,0x0C,0x06}, // ;
    {0x18,0x0C,0x06,0x03,0x06,0x0C,0x18,0x00}, // <
    {0x00,0x00,0x3F,0x00,0x00,0x3F,0x00,0x00}, // =
    {0x06,0x0C,0x18,0x30,0x18,0x0C,0x06,0x00}, // >
    {0x1E,0x33,0x30,0x18,0x0C,0x00,0x0C,0x00}, // ?
    {0x3E,0x63,0x7B,0x7B,0x7B,0x03,0x1E,0x00}, // @
    {0x0C,0x1E,0x33,0x33,0x3F,0x33,0x33,0x00}, // A
    {0x3F,0x66,0x66,0x3E,0x66,0x66,0x3F,0x00}, // B
    {0x3C,0x66,0x03,0x03,0x03,0x66,0x3C,0x00}, // C
    {0x1F,0x36,0x66,0x66,0x66,0x36,0x1F,0x00}, // D
    {0x7F,0x46,0x16,0x1E,0x16,0x46,0x7F,0x00}, // E
    {0x7F,0x46,0x16,0x1E,0x16,0x06,0x0F,0x00}, // F
    {0x3C,0x66,0x03,0x03,0x73,0x66,0x7C,0x00}, // G
    {0x33,0x33,0x33,0x3F,0x33,0x33,0x33,0x00}, // H
    {0x1E,0x0C,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // I
    {0x78,0x30,0x30,0x30,0x33,0x33,0x1E,0x00}, // J
    {0x67,0x66,0x36,0x1E,0x36,0x66,0x67,0x00}, // K
    {0x0F,0x06,0x06,0x06,0x46,0x66,0x7F,0x00}, // L
    {0x63,0x77,0x7F,0x7F,0x6B,0x63,0x63,0x00}, // M
    {0x63,0x67,0x6F,0x7B,0x73,0x63,0x63,0x00}, // N
    {0x1C,0x36,0x63,0x63,0x63,0x36,0x1C,0x00}, // O
    {0x3F,0x66,0x66,0x3E,0x06,0x06,0x0F,0x00}, // P
    {0x1E,0x33,0x33,0x33,0x3B,0x1E,0x38,0x00}, // Q
    {0x3F,0x66,0x66,0x3E,0x36,0x66,0x67,0x00}, // R
    {0x1E,0x33,0x07,0x0E,0x38,0x33,0x1E,0x00}, // S
    {0x3F,0x2D,0x0C,0x0C,0x0C,0x0C,0x1E,0x00}, // T
    {0x33,0x33,0x33,0x33,0x33,0x33,0x3F,0x00}, // U
    {0x33,0x33,0x33,0x33,0x33,0x1E,0x0C,0x00}, // V
    {0x63,0x63,0x63,0x6B,0x7F,0x77,0x63,0x00}, // W
    {0x63,0x63,0x36,0x1C,0x1C,0x36,0x63,0x00}, // X
    {0x33,0x33,0x33,0x1E,0x0C,0x0C,0x1E,0x00}, // Y
    {0x7F,0x63,0x31,0x18,0x4C,0x66,0x7F,0x00}, // Z
};

/* ── LCD helpers ─────────────────────────────────────────────────────────── */

static inline void lcd_cs_select()   { gpio_put(LCD_CS, 0); sleep_us(1); }
static inline void lcd_cs_deselect() { sleep_us(1); gpio_put(LCD_CS, 1); }
static inline void lcd_dc_command()  { gpio_put(LCD_DC, 0); }
static inline void lcd_dc_data()     { gpio_put(LCD_DC, 1); }

void lcd_write_command(uint8_t cmd) {
    lcd_dc_command();
    lcd_cs_select();
    spi_write_blocking(LCD_SPI, &cmd, 1);
    lcd_cs_deselect();
}

void lcd_write_data(uint8_t data) {
    lcd_dc_data();
    lcd_cs_select();
    spi_write_blocking(LCD_SPI, &data, 1);
    lcd_cs_deselect();
}

void lcd_reset() {
    gpio_put(LCD_RESET, 1); sleep_ms(5);
    gpio_put(LCD_RESET, 0); sleep_ms(20);
    gpio_put(LCD_RESET, 1); sleep_ms(150);
}

void lcd_init() {
    lcd_reset();
    lcd_write_command(ILI9341_SWRESET); sleep_ms(150);
    lcd_write_command(ILI9341_SLPOUT);  sleep_ms(120);
    lcd_write_command(ILI9341_PIXFMT);  lcd_write_data(0x55);
    lcd_write_command(ILI9341_MADCTL);  lcd_write_data(0x48);
    lcd_write_command(ILI9341_DISPON);  sleep_ms(100);
    printf("LCD initialized\n");
}

void lcd_set_address_window(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1) {
    lcd_write_command(ILI9341_CASET);
    lcd_write_data(x0 >> 8); lcd_write_data(x0 & 0xFF);
    lcd_write_data(x1 >> 8); lcd_write_data(x1 & 0xFF);
    lcd_write_command(ILI9341_PASET);
    lcd_write_data(y0 >> 8); lcd_write_data(y0 & 0xFF);
    lcd_write_data(y1 >> 8); lcd_write_data(y1 & 0xFF);
    lcd_write_command(ILI9341_RAMWR);
}

void lcd_fill_rect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color) {
    if (x >= LCD_WIDTH || y >= LCD_HEIGHT) return;
    if (x + w > LCD_WIDTH)  w = LCD_WIDTH  - x;
    if (y + h > LCD_HEIGHT) h = LCD_HEIGHT - y;
    uint8_t buf[2] = {color >> 8, color & 0xFF};
    lcd_set_address_window(x, y, x + w - 1, y + h - 1);
    lcd_dc_data();
    lcd_cs_select();
    for (uint32_t i = 0; i < (uint32_t)w * h; i++) {
        spi_write_blocking(LCD_SPI, buf, 2);
    }
    lcd_cs_deselect();
}

void lcd_fill_screen(uint16_t color) {
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

void lcd_draw_char(uint16_t x, uint16_t y, char c, uint16_t color, uint16_t bg, uint8_t scale) {
    if (c < 32 || c > 90) c = 32;
    const uint8_t *glyph = font8x8[c - 32];
    lcd_set_address_window(x, y, x + 8*scale - 1, y + 8*scale - 1);
    uint8_t chi = color >> 8, clo = color & 0xFF;
    uint8_t bhi = bg    >> 8, blo = bg    & 0xFF;
    lcd_dc_data();
    lcd_cs_select();
    for (uint8_t row = 0; row < 8; row++) {
        for (uint8_t sy = 0; sy < scale; sy++) {
            for (uint8_t col = 0; col < 8; col++) {
                bool on = glyph[row] & (1 << col);
                uint8_t px[2] = {on ? chi : bhi, on ? clo : blo};
                for (uint8_t sx = 0; sx < scale; sx++) {
                    spi_write_blocking(LCD_SPI, px, 2);
                }
            }
        }
    }
    lcd_cs_deselect();
}

void lcd_draw_string(uint16_t x, uint16_t y, const char *str, uint16_t color, uint16_t bg, uint8_t scale) {
    while (*str) {
        lcd_draw_char(x, y, *str, color, bg, scale);
        x += 8 * scale;
        str++;
    }
}

/* ── Space status rendering ──────────────────────────────────────────────── */

void lcd_draw_space_status(int i, bool sensor_occupied) {
    uint16_t y = 65 + i * 50;
    bool show_occupied;

    if (space_type[i] == SPACE_UNAVAILABLE) {
        lcd_fill_rect(130, y - 2, 100, 22, COLOR_ORANGE);
        lcd_draw_string(155, y + 2, "N/A", COLOR_BLACK, COLOR_ORANGE, 2);
        return;
    } else if (override_active[i]) {
        show_occupied = override_occupied[i];
    } else {
        show_occupied = sensor_occupied;
    }

    if (show_occupied) {
        lcd_fill_rect(130, y - 2, 100, 22, COLOR_RED);
        lcd_draw_string(143, y + 2, "FULL", COLOR_WHITE, COLOR_RED, 2);
    } else if (space_type[i] == SPACE_HANDICAP) {
        lcd_fill_rect(130, y - 2, 100, 22, COLOR_BLUE);
        lcd_draw_string(143, y + 2, "OPEN", COLOR_WHITE, COLOR_BLUE, 2);
    } else {
        lcd_fill_rect(130, y - 2, 100, 22, COLOR_GREEN);
        lcd_draw_string(143, y + 2, "OPEN", COLOR_BLACK, COLOR_GREEN, 2);
    }
}

void lcd_draw_parking_screen(bool occupied[NUM_SPACES]) {
    lcd_fill_screen(COLOR_BLACK);
    lcd_draw_string(20, 10, "SMART PARKING", COLOR_CYAN,  COLOR_BLACK, 2);
    lcd_draw_string(30, 35, "PURDUE IUPUI",  COLOR_GRAY,  COLOR_BLACK, 1);
    lcd_fill_rect(0, 52, LCD_WIDTH, 2, COLOR_GRAY);

    for (int i = 0; i < NUM_SPACES; i++) {
        uint16_t y = 65 + i * 50;
        char label[12];
        if (space_type[i] == SPACE_HANDICAP) {
            snprintf(label, sizeof(label), "SP%d HC", i + 1);
            lcd_draw_string(10, y, label, COLOR_CYAN,  COLOR_BLACK, 2);
        } else if (space_type[i] == SPACE_UNAVAILABLE) {
            snprintf(label, sizeof(label), "SP%d", i + 1);
            lcd_draw_string(10, y, label, COLOR_GRAY,  COLOR_BLACK, 2);
        } else {
            snprintf(label, sizeof(label), "SP%d", i + 1);
            lcd_draw_string(10, y, label, COLOR_WHITE, COLOR_BLACK, 2);
        }
        lcd_draw_space_status(i, occupied[i]);
    }

    lcd_fill_rect(0, 315, LCD_WIDTH, 2, COLOR_GRAY);

    int free_count = 0, total_usable = 0;
    for (int i = 0; i < NUM_SPACES; i++) {
        if (space_type[i] != SPACE_UNAVAILABLE) {
            total_usable++;
            if (!occupied[i]) free_count++;
        }
    }
    char summary[24];
    snprintf(summary, sizeof(summary), "AVAILABLE: %d/%d", free_count, total_usable);
    lcd_draw_string(30, 320, summary, COLOR_YELLOW, COLOR_BLACK, 1);
}

void lcd_update_changed(bool current_state[NUM_SPACES], bool force_label[NUM_SPACES]) {
    for (int i = 0; i < NUM_SPACES; i++) {
        bool display_state   = override_active[i] ? override_occupied[i] : current_state[i];
        bool display_changed = (display_state != last_state[i]) || force_label[i];
        if (!display_changed) continue;

        last_state[i] = display_state;

        if (force_label[i]) {
            uint16_t y = 65 + i * 50;
            lcd_fill_rect(0, y - 2, 128, 22, COLOR_BLACK);
            char label[12];
            if (space_type[i] == SPACE_HANDICAP) {
                snprintf(label, sizeof(label), "SP%d HC", i + 1);
                lcd_draw_string(10, y, label, COLOR_CYAN,  COLOR_BLACK, 2);
            } else if (space_type[i] == SPACE_UNAVAILABLE) {
                snprintf(label, sizeof(label), "SP%d", i + 1);
                lcd_draw_string(10, y, label, COLOR_GRAY,  COLOR_BLACK, 2);
            } else {
                snprintf(label, sizeof(label), "SP%d", i + 1);
                lcd_draw_string(10, y, label, COLOR_WHITE, COLOR_BLACK, 2);
            }
        }
        lcd_draw_space_status(i, current_state[i]);
    }

    int free_count = 0, total_usable = 0;
    for (int i = 0; i < NUM_SPACES; i++) {
        if (space_type[i] != SPACE_UNAVAILABLE) {
            total_usable++;
            bool disp = override_active[i] ? override_occupied[i] : current_state[i];
            if (!disp) free_count++;
        }
    }
    lcd_fill_rect(0, 318, LCD_WIDTH, 16, COLOR_BLACK);
    char summary[24];
    snprintf(summary, sizeof(summary), "AVAILABLE: %d/%d", free_count, total_usable);
    lcd_draw_string(30, 320, summary, COLOR_YELLOW, COLOR_BLACK, 1);
}

/* ── LED helpers ─────────────────────────────────────────────────────────── */

void led_set_color(int i, bool r, bool g, bool b) {
    gpio_put(led_pins[i][0], r);
    gpio_put(led_pins[i][1], g);
    gpio_put(led_pins[i][2], b);
}

void led_set_free(int i)         { led_set_color(i, 0, 1, 0); }
void led_set_handicap_free(int i){ led_set_color(i, 0, 0, 1); }
void led_set_occupied(int i)     { led_set_color(i, 1, 0, 0); }
void led_set_maintenance(int i)  { led_set_color(i, 1, 1, 0); }

void led_apply_state(int i, bool sensor_occupied) {
    if (space_type[i] == SPACE_UNAVAILABLE) {
        led_set_maintenance(i);
        return;
    }
    bool show_occupied = override_active[i] ? override_occupied[i] : sensor_occupied;
    if (show_occupied) {
        led_set_occupied(i);
    } else if (space_type[i] == SPACE_HANDICAP) {
        led_set_handicap_free(i);
    } else {
        led_set_free(i);
    }
}

/* ── Sensor / button / init ──────────────────────────────────────────────── */

bool space_is_occupied(int i) {
    if (space_type[i] == SPACE_UNAVAILABLE) return false;
    return !gpio_get(sensor_pins[i]);
}

void sensors_init() {
    for (int i = 0; i < NUM_SPACES; i++) {
        gpio_init(sensor_pins[i]);
        gpio_set_dir(sensor_pins[i], GPIO_IN);
    }
}

void buttons_init() {
    for (int i = 0; i < NUM_SPACES; i++) {
        gpio_init(button_pins[i]);
        gpio_set_dir(button_pins[i], GPIO_IN);
        gpio_pull_up(button_pins[i]);
    }
}

void leds_init() {
    for (int i = 0; i < NUM_SPACES; i++) {
        for (int c = 0; c < 3; c++) {
            gpio_init(led_pins[i][c]);
            gpio_set_dir(led_pins[i][c], GPIO_OUT);
            gpio_put(led_pins[i][c], 0);
        }
    }
}

void lcd_spi_init() {
    spi_init(LCD_SPI, 62500000);
    spi_set_format(LCD_SPI, 8, SPI_CPOL_0, SPI_CPHA_0, SPI_MSB_FIRST);
    gpio_set_function(LCD_MOSI,  GPIO_FUNC_SPI);
    gpio_set_function(LCD_SCK,   GPIO_FUNC_SPI);
    gpio_init(LCD_CS);    gpio_set_dir(LCD_CS,    GPIO_OUT); gpio_put(LCD_CS,    1);
    gpio_init(LCD_DC);    gpio_set_dir(LCD_DC,    GPIO_OUT); gpio_put(LCD_DC,    1);
    gpio_init(LCD_RESET); gpio_set_dir(LCD_RESET, GPIO_OUT); gpio_put(LCD_RESET, 1);
    gpio_init(LCD_LED);   gpio_set_dir(LCD_LED,   GPIO_OUT); gpio_put(LCD_LED,   1);
}

/* ── Button processing ───────────────────────────────────────────────────── */

bool process_button_for_space(int btn, int space, bool sensor_occupied, bool *force_label) {
    bool pressed = !gpio_get(button_pins[btn]);
    bool changed = false;
    *force_label = false;

    if (pressed && !btn_was_pressed[btn]) {
        btn_press_time[btn]  = to_ms_since_boot(get_absolute_time());
        btn_was_pressed[btn] = true;
        btn_hold_fired[btn]  = false;
    }

    if (pressed && btn_was_pressed[btn] && !btn_hold_fired[btn]) {
        uint32_t held_ms = to_ms_since_boot(get_absolute_time()) - btn_press_time[btn];
        if (held_ms >= HOLD_THRESHOLD_MS) {
            btn_hold_fired[btn] = true;
            if (space_type[space] == SPACE_UNAVAILABLE) {
                space_type[space]    = saved_type[space];
                override_active[space] = false;
            } else {
                saved_type[space]    = space_type[space];
                space_type[space]    = SPACE_UNAVAILABLE;
                override_active[space] = false;
            }
            *force_label = true;
            changed = true;
        }
    }

    if (!pressed && btn_was_pressed[btn]) {
        uint32_t held_ms = to_ms_since_boot(get_absolute_time()) - btn_press_time[btn];
        btn_was_pressed[btn] = false;
        if (!btn_hold_fired[btn] && held_ms >= DEBOUNCE_MS) {
            if (space_type[space] == SPACE_UNAVAILABLE) {
                space_type[space]      = saved_type[space];
                override_active[space] = false;
                *force_label           = true;
            } else if (!override_active[space]) {
                override_active[space]   = true;
                override_occupied[space] = !sensor_occupied;
            } else {
                override_active[space] = false;
            }
            changed = true;
        }
    }
    return changed;
}

/* ── Analytics ───────────────────────────────────────────────────────────── */

void print_parking_stats(bool current_occupied[NUM_SPACES]) {
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (now - last_print_time < 5000) return;
    last_print_time = now;

    printf("\n=== Parking System Status ===\n");
    int free_count  = 0;
    int recommended = -1;
    uint32_t best_avg_sec = 0;

    for (int i = 0; i < NUM_SPACES; i++) {
        bool     is_occupied       = current_occupied[i];
        uint32_t display_total_ms  = total_occupied_ms[i];
        uint32_t display_count     = occupation_count[i];

        if (is_occupied) {
            uint32_t live_ms = (uint32_t)(absolute_time_diff_us(
                occupation_start_time[i], get_absolute_time()) / 1000);
            if (live_ms > 0) {
                display_total_ms += live_ms;
                display_count++;
            }
        }

        uint32_t avg_sec = (display_count > 0)
            ? (display_total_ms / display_count) / 1000
            : 0;

        if (space_type[i] == SPACE_UNAVAILABLE) {
            printf("Space %d (N/A)   : UNAVAILABLE\n", i + 1);
        } else {
            printf("Space %d (%-6s): %-8s | Avg occupied: %3lu sec (%lu session%s)\n",
                i + 1,
                (space_type[i] == SPACE_HANDICAP) ? "HC" : "Normal",
                is_occupied ? "OCCUPIED" : "FREE",
                avg_sec,
                display_count,
                display_count == 1 ? "" : "s");
        }

        if (!is_occupied) free_count++;
        if (!is_occupied && space_type[i] == SPACE_NORMAL) {
            if (display_count > 0 && avg_sec > best_avg_sec) {
                best_avg_sec = avg_sec;
                recommended  = i;
            }
        }
    }

    if (recommended == -1) {
        for (int i = 0; i < NUM_SPACES; i++) {
            if (!current_occupied[i] && space_type[i] == SPACE_NORMAL) {
                recommended = i;
                break;
            }
        }
    }

    printf("-----------------------------\n");
    if (recommended != -1) {
        uint32_t rec_count = occupation_count[recommended];
        uint32_t rec_avg   = (rec_count > 0)
            ? (total_occupied_ms[recommended] / rec_count) / 1000
            : 0;
        if (rec_count > 0) {
            printf(">> RECOMMENDED: Space %d (avg %lu sec/visit — lowest turnover, least congestion)\n",
                recommended + 1, rec_avg);
        } else {
            printf(">> RECOMMENDED: Space %d (no history yet — first available normal space)\n",
                recommended + 1);
        }
    } else {
        printf(">> No free standard spaces available.\n");
    }
    printf("Free spaces: %d/%d\n", free_count, NUM_SPACES);
    printf("=============================\n\n");
}

/* ── Main ────────────────────────────────────────────────────────────────── */

int main() {
    stdio_init_all();
    sleep_ms(3000);
    printf("=== Smart Parking System Started ===\n");

    sensors_init();
    buttons_init();
    leds_init();
    lcd_spi_init();
    lcd_init();

    bool current_state[NUM_SPACES];
    for (int i = 0; i < NUM_SPACES; i++) {
        current_state[i]          = space_is_occupied(i);
        last_state[i]             = current_state[i];
        occupation_start_time[i]  = get_absolute_time();
    }
    for (int i = 0; i < NUM_SPACES; i++) {
        led_apply_state(i, current_state[i]);
    }
    lcd_draw_parking_screen(current_state);
    printf("System ready. Console will show stats every 5 seconds.\n\n");

    while (true) {
        bool changed = false;
        bool force_label[NUM_SPACES] = {false};
        bool sensor_occupied[NUM_SPACES];

        for (int i = 0; i < NUM_SPACES; i++) {
            sensor_occupied[i] = space_is_occupied(i);
        }

        for (int btn = 0; btn < NUM_SPACES; btn++) {
            int  space = button_controls_space[btn];
            bool lbl   = false;
            if (process_button_for_space(btn, space, sensor_occupied[space], &lbl)) {
                changed           = true;
                force_label[space] = lbl;
            }
        }

        for (int i = 0; i < NUM_SPACES; i++) {
            if (sensor_occupied[i] && !current_state[i]) {
                occupation_start_time[i] = get_absolute_time();
            } else if (!sensor_occupied[i] && current_state[i]) {
                uint32_t duration_ms = absolute_time_diff_us(
                    occupation_start_time[i], get_absolute_time()) / 1000;
                if (duration_ms > 1000) {
                    total_occupied_ms[i] += duration_ms;
                    occupation_count[i]++;
                }
            }
            led_apply_state(i, sensor_occupied[i]);
            if (sensor_occupied[i] != current_state[i]) {
                current_state[i] = sensor_occupied[i];
                changed          = true;
            }
        }

        if (changed) {
            lcd_update_changed(current_state, force_label);
        }

        print_parking_stats(current_state);
        sleep_ms(50);
    }

    return 0;
}
