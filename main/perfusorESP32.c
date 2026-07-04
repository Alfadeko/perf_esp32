#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <math.h>

#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "driver/spi_master.h"
#include "driver/touch_sens.h"
#include "esp_check.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "soc/gpio_struct.h"
#include "soc/touch_sensor_channel.h"

#define LCD_HOST SPI2_HOST
#define LCD_WIDTH 240
#define LCD_HEIGHT 135

#define PIN_LCD_MOSI 19
#define PIN_LCD_CLK 18
#define PIN_LCD_CS 5
#define PIN_LCD_DC 16
#define PIN_LCD_RST 23
#define PIN_LCD_BL 4

#define PIN_STEP 25
#define PIN_DIR 26
#define PIN_EN 27

#define TOUCH_ADV_CHAN TOUCH_PAD_GPIO13_CHANNEL
#define TOUCH_RUN_CHAN TOUCH_PAD_GPIO15_CHANNEL
#define TOUCH_BACK_CHAN TOUCH_PAD_GPIO2_CHANNEL

#define STEPS_PER_MM 800.0f
#define MIN_SPEED_MM_S 0.10f
#define MAX_SPEED_MM_S 20.00f
#define MAX_DISTANCE_MM 999.9f
#define ACCEL_MM_S2 5.0f

#define MAGIC_CFG 0x50465253u

typedef struct {
    float speed_fwd;
    float dist_fwd;
    float speed_back;
    float dist_back;
    float pause_s;
    uint32_t magic;
} perf_cfg_t;

typedef struct {
    int chan_id;
    touch_channel_handle_t chan_handle;
    uint16_t baseline;
    uint16_t threshold;
    bool pressed;
    bool just_pressed;
    bool just_released;
    int64_t press_start_ms;
    uint32_t last_hold_ms;
} touch_btn_t;

typedef enum {
    UI_BOOT = 0,
    UI_MAIN,
    UI_CONFIG,
    UI_EDIT,
    UI_ABOUT,
} ui_mode_t;

typedef enum {
    MENU_SPEED_FWD = 0,
    MENU_DIST_FWD,
    MENU_SPEED_BACK,
    MENU_DIST_BACK,
    MENU_PAUSE,
    MENU_ABOUT,
    MENU_SAVE_EXIT,
    MENU_CANCEL,
    MENU_COUNT,
} menu_item_t;

static const char *TAG = "perfusor";
static esp_lcd_panel_handle_t s_panel;
static perf_cfg_t s_cfg;
static perf_cfg_t s_cfg_edit;
static uint32_t s_cycles;
static ui_mode_t s_mode;
static menu_item_t s_menu_idx;
static int s_edit_digit_idx;
static char s_edit_buf[8];
static bool s_motor_ready;
static float s_position_pct;
static touch_sensor_handle_t s_touch_handle;
static gptimer_handle_t s_step_timer;
static volatile bool s_step_active;
static volatile bool s_step_high;
static volatile uint32_t s_step_target_pulses;
static volatile uint32_t s_step_done_pulses;
static QueueHandle_t s_motor_cmd_queue;
static TaskHandle_t s_motor_waiter_task;
static volatile bool s_motor_last_cancelled;

typedef struct {
    float distance_mm;
    float target_speed_mm_s;
    bool forward;
    bool with_progress;
} motor_move_cmd_t;

static touch_btn_t s_btn_adv = {.chan_id = TOUCH_ADV_CHAN};
static touch_btn_t s_btn_run = {.chan_id = TOUCH_RUN_CHAN};
static touch_btn_t s_btn_back = {.chan_id = TOUCH_BACK_CHAN};

static inline uint16_t rgb565(uint8_t r, uint8_t g, uint8_t b)
{
    return (uint16_t)(((r & 0xF8u) << 8) | ((g & 0xFCu) << 3) | (b >> 3));
}

static const uint8_t font5x7[95][5] = {
    {0x00,0x00,0x00,0x00,0x00},{0x00,0x00,0x5F,0x00,0x00},{0x00,0x07,0x00,0x07,0x00},{0x14,0x7F,0x14,0x7F,0x14},{0x24,0x2A,0x7F,0x2A,0x12},{0x23,0x13,0x08,0x64,0x62},{0x36,0x49,0x55,0x22,0x50},{0x00,0x05,0x03,0x00,0x00},{0x00,0x1C,0x22,0x41,0x00},{0x00,0x41,0x22,0x1C,0x00},{0x14,0x08,0x3E,0x08,0x14},{0x08,0x08,0x3E,0x08,0x08},{0x00,0x50,0x30,0x00,0x00},{0x08,0x08,0x08,0x08,0x08},{0x00,0x60,0x60,0x00,0x00},{0x20,0x10,0x08,0x04,0x02},
    {0x3E,0x51,0x49,0x45,0x3E},{0x00,0x42,0x7F,0x40,0x00},{0x42,0x61,0x51,0x49,0x46},{0x21,0x41,0x45,0x4B,0x31},{0x18,0x14,0x12,0x7F,0x10},{0x27,0x45,0x45,0x45,0x39},{0x3C,0x4A,0x49,0x49,0x30},{0x01,0x71,0x09,0x05,0x03},{0x36,0x49,0x49,0x49,0x36},{0x06,0x49,0x49,0x29,0x1E},{0x00,0x36,0x36,0x00,0x00},{0x00,0x56,0x36,0x00,0x00},{0x08,0x14,0x22,0x41,0x00},{0x14,0x14,0x14,0x14,0x14},{0x00,0x41,0x22,0x14,0x08},{0x02,0x01,0x51,0x09,0x06},
    {0x32,0x49,0x79,0x41,0x3E},{0x7E,0x11,0x11,0x11,0x7E},{0x7F,0x49,0x49,0x49,0x36},{0x3E,0x41,0x41,0x41,0x22},{0x7F,0x41,0x41,0x22,0x1C},{0x7F,0x49,0x49,0x49,0x41},{0x7F,0x09,0x09,0x09,0x01},{0x3E,0x41,0x49,0x49,0x7A},{0x7F,0x08,0x08,0x08,0x7F},{0x00,0x41,0x7F,0x41,0x00},{0x20,0x40,0x41,0x3F,0x01},{0x7F,0x08,0x14,0x22,0x41},{0x7F,0x40,0x40,0x40,0x40},{0x7F,0x02,0x0C,0x02,0x7F},{0x7F,0x04,0x08,0x10,0x7F},{0x3E,0x41,0x41,0x41,0x3E},
    {0x7F,0x09,0x09,0x09,0x06},{0x3E,0x41,0x51,0x21,0x5E},{0x7F,0x09,0x19,0x29,0x46},{0x46,0x49,0x49,0x49,0x31},{0x01,0x01,0x7F,0x01,0x01},{0x3F,0x40,0x40,0x40,0x3F},{0x1F,0x20,0x40,0x20,0x1F},{0x7F,0x20,0x18,0x20,0x7F},{0x63,0x14,0x08,0x14,0x63},{0x03,0x04,0x78,0x04,0x03},{0x61,0x51,0x49,0x45,0x43},{0x00,0x00,0x7F,0x41,0x41},{0x02,0x04,0x08,0x10,0x20},{0x41,0x41,0x7F,0x00,0x00},{0x04,0x02,0x01,0x02,0x04},{0x80,0x80,0x80,0x80,0x80},
    {0x00,0x03,0x05,0x00,0x00},{0x20,0x54,0x54,0x54,0x78},{0x7F,0x48,0x44,0x44,0x38},{0x38,0x44,0x44,0x44,0x20},{0x38,0x44,0x44,0x48,0x7F},{0x38,0x54,0x54,0x54,0x18},{0x08,0x7E,0x09,0x01,0x02},{0x0C,0x52,0x52,0x52,0x3E},{0x7F,0x08,0x04,0x04,0x78},{0x00,0x44,0x7D,0x40,0x00},{0x20,0x40,0x44,0x3D,0x00},{0x7F,0x10,0x28,0x44,0x00},{0x00,0x41,0x7F,0x40,0x00},{0x7C,0x04,0x18,0x04,0x78},{0x7C,0x08,0x04,0x04,0x78},{0x38,0x44,0x44,0x44,0x38},
    {0x7C,0x14,0x14,0x14,0x08},{0x08,0x14,0x14,0x18,0x7C},{0x7C,0x08,0x04,0x04,0x08},{0x48,0x54,0x54,0x54,0x20},{0x04,0x3F,0x44,0x40,0x20},{0x3C,0x40,0x40,0x20,0x7C},{0x1C,0x20,0x40,0x20,0x1C},{0x3C,0x40,0x30,0x40,0x3C},{0x44,0x28,0x10,0x28,0x44},{0x0C,0x50,0x50,0x50,0x3C},{0x44,0x64,0x54,0x4C,0x44},{0x00,0x08,0x36,0x41,0x00},{0x00,0x00,0x7F,0x00,0x00},{0x00,0x41,0x36,0x08,0x00},{0x08,0x04,0x08,0x10,0x08}
};

static void lcd_fill_rect(int x, int y, int w, int h, uint16_t color)
{
    if (w <= 0 || h <= 0) {
        return;
    }
    if (x < 0) {
        w += x;
        x = 0;
    }
    if (y < 0) {
        h += y;
        y = 0;
    }
    if (x + w > LCD_WIDTH) {
        w = LCD_WIDTH - x;
    }
    if (y + h > LCD_HEIGHT) {
        h = LCD_HEIGHT - y;
    }
    if (w <= 0 || h <= 0) {
        return;
    }

    static uint16_t line[LCD_WIDTH];
    for (int i = 0; i < w; ++i) {
        line[i] = color;
    }
    for (int row = 0; row < h; ++row) {
        esp_lcd_panel_draw_bitmap(s_panel, x, y + row, x + w, y + row + 1, line);
    }
}

static void lcd_clear(uint16_t color)
{
    lcd_fill_rect(0, 0, LCD_WIDTH, LCD_HEIGHT, color);
}

static void lcd_draw_char(int x, int y, char c, uint16_t fg, uint16_t bg, int scale)
{
    if (c < 32 || c > 126) {
        c = '?';
    }
    const uint8_t *g = font5x7[(int)c - 32];
    for (int col = 0; col < 5; ++col) {
        uint8_t bits = g[col];
        for (int row = 0; row < 7; ++row) {
            bool on = (bits >> row) & 0x01;
            uint16_t color = on ? fg : bg;
            lcd_fill_rect(x + col * scale, y + row * scale, scale, scale, color);
        }
    }
    lcd_fill_rect(x + 5 * scale, y, scale, 7 * scale, bg);
}

static void lcd_draw_text(int x, int y, const char *txt, uint16_t fg, uint16_t bg, int scale)
{
    int cx = x;
    while (*txt) {
        char c = *txt++;
        if (c == '\n') {
            y += 8 * scale;
            cx = x;
            continue;
        }
        lcd_draw_char(cx, y, c, fg, bg, scale);
        cx += 6 * scale;
    }
}

static void lcd_draw_text_upper(int x, int y, const char *txt, uint16_t fg, uint16_t bg, int scale)
{
    char buf[64];
    size_t n = strlen(txt);
    if (n > sizeof(buf) - 1) {
        n = sizeof(buf) - 1;
    }
    for (size_t i = 0; i < n; ++i) {
        buf[i] = (char)toupper((unsigned char)txt[i]);
    }
    buf[n] = '\0';
    lcd_draw_text(x, y, buf, fg, bg, scale);
}

static void draw_boot_screen(void)
{
    const uint16_t bg = rgb565(8, 14, 25);
    const uint16_t fg = rgb565(240, 240, 210);
    const uint16_t ac = rgb565(70, 190, 250);

    lcd_clear(bg);
    lcd_draw_text_upper(32, 50, "eugeidea 2026", fg, bg, 2);

    for (int i = 0; i < 100; i += 5) {
        lcd_fill_rect(30, 95, 180, 8, rgb565(20, 30, 45));
        lcd_fill_rect(30, 95, (180 * i) / 100, 8, ac);
        vTaskDelay(pdMS_TO_TICKS(35));
    }
}

static void draw_progress_percent(float pct);

static void draw_main_screen(uint32_t cycles)
{
    const uint16_t bg = rgb565(12, 16, 20);
    const uint16_t fg = rgb565(245, 245, 245);
    char line[32];
    int scale = 7;
    int text_w;
    int x;

    lcd_clear(bg);
    draw_progress_percent(s_position_pct);

    snprintf(line, sizeof(line), "%lu", (unsigned long)cycles);
    text_w = (int)strlen(line) * 6 * scale;
    x = LCD_WIDTH - text_w - 8;
    if (x < 96) {
        x = 96;
    }
    lcd_draw_text(x, 42, line, fg, bg, scale);

    lcd_fill_rect(0, 108, LCD_WIDTH, 27, bg);
}

static void draw_progress_percent(float pct)
{
    char t[8];
    const uint16_t bg = rgb565(12, 16, 20);
    if (pct < 0.0f) {
        pct = 0.0f;
    }
    if (pct > 100.0f) {
        pct = 100.0f;
    }
    s_position_pct = pct;
    snprintf(t, sizeof(t), "%3d%%", (int)pct);
    lcd_fill_rect(4, 4, 84, 26, bg);
    lcd_draw_text_upper(4, 4, t, rgb565(60, 220, 150), bg, 3);
}

static void draw_main_touch_feedback(bool adv_pressed, bool run_pressed, bool back_pressed)
{
    const uint16_t bg = rgb565(12, 16, 20);
    lcd_fill_rect(0, 108, LCD_WIDTH, 27, bg);
    lcd_draw_text_upper(8, 114, "ADEL", adv_pressed ? rgb565(255, 215, 90) : rgb565(140, 160, 180), bg, 1);
    lcd_draw_text_upper(88, 114, "MARCHA", run_pressed ? rgb565(255, 110, 95) : rgb565(140, 160, 180), bg, 1);
    lcd_draw_text_upper(188, 114, "ATRAS", back_pressed ? rgb565(95, 200, 255) : rgb565(140, 160, 180), bg, 1);
}

static const char *menu_name(menu_item_t item)
{
    static const char *names[MENU_COUNT] = {
        "Velocidad avance",
        "Distancia avance",
        "Velocidad retroceso",
        "Distancia retroceso",
        "Pausa tras avance",
        "Acerca de",
        "Guardar y salir",
        "Cancelar",
    };
    return names[item];
}

static void draw_config_screen(void)
{
    char v[32];
    const uint16_t bg = rgb565(6, 18, 24);
    const uint16_t fg = rgb565(220, 240, 255);
    const uint16_t ac = rgb565(100, 220, 255);

    lcd_clear(bg);
    lcd_draw_text_upper(8, 8, "CONFIGURACION", ac, bg, 2);
    lcd_draw_text_upper(8, 42, menu_name(s_menu_idx), fg, bg, 2);

    switch (s_menu_idx) {
    case MENU_SPEED_FWD:
        snprintf(v, sizeof(v), "%.2f mm/s", s_cfg_edit.speed_fwd);
        break;
    case MENU_DIST_FWD:
        snprintf(v, sizeof(v), "%.1f mm", s_cfg_edit.dist_fwd);
        break;
    case MENU_SPEED_BACK:
        snprintf(v, sizeof(v), "%.2f mm/s", s_cfg_edit.speed_back);
        break;
    case MENU_DIST_BACK:
        snprintf(v, sizeof(v), "%.1f mm", s_cfg_edit.dist_back);
        break;
    case MENU_PAUSE:
        snprintf(v, sizeof(v), "%.1f s", s_cfg_edit.pause_s);
        break;
    case MENU_ABOUT:
        snprintf(v, sizeof(v), "Version 2026");
        break;
    case MENU_SAVE_EXIT:
        snprintf(v, sizeof(v), "Guardar");
        break;
    default:
        snprintf(v, sizeof(v), "Descartar cambios");
        break;
    }

    lcd_draw_text_upper(8, 82, v, rgb565(255, 240, 120), bg, 2);
    lcd_draw_text_upper(8, 120, "A/B rueda, marcha entra", rgb565(130, 170, 190), bg, 1);
}

static void draw_about_screen(void)
{
    uint16_t bg = rgb565(10, 10, 26);
    lcd_clear(bg);
    lcd_draw_text_upper(18, 18, "PERFUSOR ESP32", rgb565(200, 235, 255), bg, 2);
    lcd_draw_text_upper(18, 55, "eugeidea 2026", rgb565(120, 230, 255), bg, 2);
    lcd_draw_text_upper(18, 95, "MARCHA para volver", rgb565(160, 170, 220), bg, 1);
}

static void draw_edit_screen(void)
{
    uint16_t bg = rgb565(28, 20, 10);
    lcd_clear(bg);
    lcd_draw_text_upper(8, 8, "EDITAR VALOR", rgb565(255, 225, 140), bg, 2);
    lcd_draw_text(36, 54, s_edit_buf, rgb565(250, 250, 250), bg, 4);

    int pos = s_edit_digit_idx;
    int x = 36 + (6 * 4 * pos);
    lcd_fill_rect(x, 90, 18, 4, rgb565(255, 180, 80));
    lcd_draw_text_upper(8, 112, "A/B digito, marcha siguiente", rgb565(200, 180, 130), bg, 1);
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) {
        return lo;
    }
    if (v > hi) {
        return hi;
    }
    return v;
}

static void cfg_set_defaults(perf_cfg_t *cfg)
{
    cfg->speed_fwd = 2.00f;
    cfg->dist_fwd = 20.0f;
    cfg->speed_back = 2.00f;
    cfg->dist_back = 20.0f;
    cfg->pause_s = 0.5f;
    cfg->magic = MAGIC_CFG;
}

static esp_err_t cfg_save(const perf_cfg_t *cfg)
{
    nvs_handle_t h;
    ESP_RETURN_ON_ERROR(nvs_open("perfusor", NVS_READWRITE, &h), TAG, "nvs_open save");
    esp_err_t err = nvs_set_blob(h, "cfg", cfg, sizeof(*cfg));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    return err;
}

static void cfg_load(perf_cfg_t *cfg)
{
    size_t sz = sizeof(*cfg);
    nvs_handle_t h;
    if (nvs_open("perfusor", NVS_READONLY, &h) != ESP_OK) {
        cfg_set_defaults(cfg);
        return;
    }
    if (nvs_get_blob(h, "cfg", cfg, &sz) != ESP_OK || sz != sizeof(*cfg) || cfg->magic != MAGIC_CFG) {
        cfg_set_defaults(cfg);
    }
    nvs_close(h);

    cfg->speed_fwd = clampf(cfg->speed_fwd, MIN_SPEED_MM_S, MAX_SPEED_MM_S);
    cfg->speed_back = clampf(cfg->speed_back, MIN_SPEED_MM_S, MAX_SPEED_MM_S);
    cfg->dist_fwd = clampf(cfg->dist_fwd, 0.0f, MAX_DISTANCE_MM);
    cfg->dist_back = clampf(cfg->dist_back, 0.0f, MAX_DISTANCE_MM);
    cfg->pause_s = clampf(cfg->pause_s, 0.0f, 60.0f);
    cfg->magic = MAGIC_CFG;
}

static esp_err_t lcd_init(void)
{
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_LCD_CLK,
        .mosi_io_num = PIN_LCD_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_WIDTH * 20 * sizeof(uint16_t),
    };
    ESP_RETURN_ON_ERROR(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO), TAG, "spi_bus_initialize");

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_LCD_DC,
        .cs_gpio_num = PIN_LCD_CS,
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io), TAG, "new_panel_io");

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = PIN_LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
        .bits_per_pixel = 16,
    };
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789(io, &panel_config, &s_panel), TAG, "new_panel_st7789");

    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_panel), TAG, "panel_reset");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_panel), TAG, "panel_init");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_invert_color(s_panel, true), TAG, "panel_invert");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_swap_xy(s_panel, true), TAG, "panel_swapxy");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_mirror(s_panel, true, false), TAG, "panel_mirror");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(s_panel, 40, 53), TAG, "panel_gap");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_panel, true), TAG, "panel_on");

    gpio_config_t bl_cfg = {
        .pin_bit_mask = 1ULL << PIN_LCD_BL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&bl_cfg), TAG, "bl gpio");
    gpio_set_level(PIN_LCD_BL, 1);
    return ESP_OK;
}

static esp_err_t touch_init_and_calibrate(void)
{
    static touch_sensor_sample_config_t sample_cfg =
        TOUCH_SENSOR_V1_DEFAULT_SAMPLE_CONFIG(2.0f, TOUCH_VOLT_LIM_L_0V5, TOUCH_VOLT_LIM_H_2V7);
    static touch_sensor_config_t sens_cfg = TOUCH_SENSOR_DEFAULT_BASIC_CONFIG(1, &sample_cfg);
    static touch_channel_config_t chan_cfg = {
        .abs_active_thresh = {0},
        .charge_speed = TOUCH_CHARGE_SPEED_4,
        .init_charge_volt = TOUCH_INIT_CHARGE_VOLT_DEFAULT,
        .group = TOUCH_CHAN_TRIG_GROUP_BOTH,
    };
    static touch_sensor_filter_config_t filter_cfg = TOUCH_SENSOR_DEFAULT_FILTER_CONFIG();

    ESP_RETURN_ON_ERROR(touch_sensor_new_controller(&sens_cfg, &s_touch_handle), TAG, "touch new controller");
    ESP_RETURN_ON_ERROR(touch_sensor_new_channel(s_touch_handle, s_btn_adv.chan_id, &chan_cfg, &s_btn_adv.chan_handle), TAG, "touch adv ch");
    ESP_RETURN_ON_ERROR(touch_sensor_new_channel(s_touch_handle, s_btn_run.chan_id, &chan_cfg, &s_btn_run.chan_handle), TAG, "touch run ch");
    ESP_RETURN_ON_ERROR(touch_sensor_new_channel(s_touch_handle, s_btn_back.chan_id, &chan_cfg, &s_btn_back.chan_handle), TAG, "touch back ch");
    ESP_RETURN_ON_ERROR(touch_sensor_enable(s_touch_handle), TAG, "touch enable");
    ESP_RETURN_ON_ERROR(touch_sensor_config_filter(s_touch_handle, &filter_cfg), TAG, "touch filter");
    ESP_RETURN_ON_ERROR(touch_sensor_start_continuous_scanning(s_touch_handle), TAG, "touch start scan");

    vTaskDelay(pdMS_TO_TICKS(350));

    touch_btn_t *btns[] = {&s_btn_adv, &s_btn_run, &s_btn_back};
    for (size_t i = 0; i < sizeof(btns) / sizeof(btns[0]); ++i) {
        uint32_t acc = 0;
        int valid_samples = 0;
        for (int attempts = 0; attempts < 64 && valid_samples < 16; ++attempts) {
            uint32_t val32 = 0;
            esp_err_t err = touch_channel_read_data(btns[i]->chan_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, &val32);
            if (err != ESP_OK) {
                err = touch_channel_read_data(btns[i]->chan_handle, TOUCH_CHAN_DATA_TYPE_RAW, &val32);
            }
            if (err != ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(10));
                continue;
            }
            uint16_t val = (uint16_t)((val32 > 0xFFFFu) ? 0xFFFFu : val32);
            acc += val;
            valid_samples++;
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        if (valid_samples == 0) {
            btns[i]->baseline = 1;
            btns[i]->threshold = 1;
            ESP_LOGW(TAG, "touch chan %d without valid samples, using safe defaults", btns[i]->chan_id);
        } else {
            btns[i]->baseline = (uint16_t)(acc / (uint32_t)valid_samples);
            btns[i]->threshold = (uint16_t)(btns[i]->baseline * 9 / 10);
        }
        if (btns[i]->threshold == 0) {
            btns[i]->threshold = 1;
        }
        btns[i]->pressed = false;
        btns[i]->just_pressed = false;
        btns[i]->just_released = false;
        btns[i]->press_start_ms = 0;
        btns[i]->last_hold_ms = 0;
        ESP_LOGI(TAG, "touch chan %d baseline=%u threshold=%u samples=%d", btns[i]->chan_id, btns[i]->baseline, btns[i]->threshold, valid_samples);
    }
    return ESP_OK;
}

static void touch_update_btn(touch_btn_t *b)
{
    uint32_t val32 = 0;
    esp_err_t err = touch_channel_read_data(b->chan_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, &val32);
    if (err != ESP_OK) {
        if (touch_channel_read_data(b->chan_handle, TOUCH_CHAN_DATA_TYPE_RAW, &val32) != ESP_OK) {
            b->just_pressed = false;
            b->just_released = false;
            return;
        }
    }
    uint16_t val = (uint16_t)((val32 > 0xFFFFu) ? 0xFFFFu : val32);
    bool now = val < b->threshold;

    b->just_pressed = (!b->pressed && now);
    b->just_released = (b->pressed && !now);
    b->pressed = now;

    int64_t now_ms = esp_timer_get_time() / 1000;
    if (b->just_pressed) {
        b->press_start_ms = now_ms;
        b->last_hold_ms = 0;
    }
    if (b->pressed) {
        b->last_hold_ms = (uint32_t)(now_ms - b->press_start_ms);
    }
}

static void touch_poll_all(void)
{
    touch_update_btn(&s_btn_adv);
    touch_update_btn(&s_btn_run);
    touch_update_btn(&s_btn_back);
}

static void motor_set_enable(bool en)
{
    if (!s_motor_ready) {
        return;
    }
    gpio_set_level(PIN_EN, en ? 0 : 1);
}

static bool IRAM_ATTR step_timer_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_ctx)
{
    (void)timer;
    (void)edata;
    (void)user_ctx;

    if (!s_step_active) {
        return false;
    }

    if (!s_step_high) {
        GPIO.out_w1ts = (1UL << PIN_STEP);
        s_step_high = true;
    } else {
        GPIO.out_w1tc = (1UL << PIN_STEP);
        s_step_high = false;
        s_step_done_pulses++;
        if (s_step_done_pulses >= s_step_target_pulses) {
            s_step_active = false;
        }
    }

    return false;
}

static bool motor_timer_set_period(uint32_t period_us)
{
    if (!s_motor_ready || s_step_timer == NULL) {
        return false;
    }

    if (period_us < 8) {
        period_us = 8;
    }
    uint64_t half_period_us = period_us / 2;
    if (half_period_us < 2) {
        half_period_us = 2;
    }

    gptimer_alarm_config_t alarm_cfg = {
        .reload_count = 0,
        .alarm_count = half_period_us,
        .flags.auto_reload_on_alarm = true,
    };
    return gptimer_set_alarm_action(s_step_timer, &alarm_cfg) == ESP_OK;
}

static bool auto_cycle_cancel_requested(void)
{
    static int64_t last_poll_ms = 0;
    static int64_t hold_start_ms = 0;
    static bool was_pressed = false;

    int64_t now_ms = esp_timer_get_time() / 1000;
    if ((now_ms - last_poll_ms) < 20) {
        return false;
    }
    last_poll_ms = now_ms;

    uint32_t val32 = 0;
    esp_err_t err = touch_channel_read_data(s_btn_run.chan_handle, TOUCH_CHAN_DATA_TYPE_SMOOTH, &val32);
    if (err != ESP_OK) {
        err = touch_channel_read_data(s_btn_run.chan_handle, TOUCH_CHAN_DATA_TYPE_RAW, &val32);
    }
    if (err != ESP_OK) {
        return false;
    }

    uint16_t val = (uint16_t)((val32 > 0xFFFFu) ? 0xFFFFu : val32);
    bool pressed = val < s_btn_run.threshold;
    if (pressed) {
        if (!was_pressed) {
            hold_start_ms = now_ms;
        }
        was_pressed = true;
        return (now_ms - hold_start_ms) >= 1000;
    }

    was_pressed = false;
    hold_start_ms = 0;
    return false;
}

static bool wait_with_cancel(uint32_t delay_ms)
{
    int64_t start_ms = esp_timer_get_time() / 1000;
    while ((uint32_t)((esp_timer_get_time() / 1000) - start_ms) < delay_ms) {
        if (auto_cycle_cancel_requested()) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
    return false;
}

static bool motor_execute_move(float distance_mm, float target_speed_mm_s, bool forward, bool with_progress)
{
    uint32_t steps = (uint32_t)(distance_mm * STEPS_PER_MM + 0.5f);
    if (steps == 0) {
        return false;
    }

    if (!s_motor_ready) {
        if (with_progress) {
            draw_progress_percent(forward ? 100.0f : 0.0f);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
        return false;
    }

    float target = clampf(target_speed_mm_s, MIN_SPEED_MM_S, MAX_SPEED_MM_S);
    float total_mm = (float)steps / STEPS_PER_MM;
    uint32_t done_steps = 0;
    int last_drawn_pct = -1;
    uint32_t last_period_us = 0;
    bool cancelled = false;

    gpio_set_level(PIN_DIR, forward ? 1 : 0);
    motor_set_enable(true);

    s_step_target_pulses = steps;
    s_step_done_pulses = 0;
    s_step_high = false;
    s_step_active = true;
    GPIO.out_w1tc = (1UL << PIN_STEP);

    while (done_steps < steps && s_step_active) {
        if (auto_cycle_cancel_requested()) {
            cancelled = true;
            s_step_active = false;
            break;
        }

        done_steps = s_step_done_pulses;
        float x_mm = (float)done_steps / STEPS_PER_MM;
        float remain_mm = total_mm - x_mm;
        float v_acc = sqrtf(2.0f * ACCEL_MM_S2 * fmaxf(x_mm, 0.0f));
        float v_dec = sqrtf(2.0f * ACCEL_MM_S2 * fmaxf(remain_mm, 0.0f));
        float cur = fminf(target, fminf(v_acc, v_dec));
        if (cur < MIN_SPEED_MM_S) {
            cur = MIN_SPEED_MM_S;
        }

        float freq = cur * STEPS_PER_MM;
        if (freq < 1.0f) {
            freq = 1.0f;
        }
        uint32_t period_us = (uint32_t)(1000000.0f / freq);
        if (period_us < 8) {
            period_us = 8;
        }

        if (period_us != last_period_us) {
            if (!motor_timer_set_period(period_us)) {
                cancelled = true;
                s_step_active = false;
                break;
            }
            if (last_period_us == 0) {
                if (gptimer_set_raw_count(s_step_timer, 0) != ESP_OK || gptimer_start(s_step_timer) != ESP_OK) {
                    cancelled = true;
                    s_step_active = false;
                    break;
                }
            }
            last_period_us = period_us;
        }

        if (with_progress) {
            float pct = (100.0f * (float)done_steps) / (float)steps;
            if (!forward) {
                pct = 100.0f - pct;
            }
            int pct_i = (int)pct;
            if (pct_i != last_drawn_pct) {
                draw_progress_percent(pct);
                last_drawn_pct = pct_i;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(1));
    }

    s_step_active = false;
    gptimer_stop(s_step_timer);
    GPIO.out_w1tc = (1UL << PIN_STEP);
    motor_set_enable(false);

    if (with_progress && !cancelled) {
        draw_progress_percent(forward ? 100.0f : 0.0f);
    }
    return cancelled;
}

static void motor_jog_while_pressed(bool forward, float target_speed_mm_s)
{
    if (!s_motor_ready || s_step_timer == NULL) {
        return;
    }

    float target = clampf(target_speed_mm_s, MIN_SPEED_MM_S, MAX_SPEED_MM_S);
    uint32_t last_period_us = 0;
    int64_t start_us = esp_timer_get_time();

    gpio_set_level(PIN_DIR, forward ? 1 : 0);
    motor_set_enable(true);

    s_step_target_pulses = UINT32_MAX;
    s_step_done_pulses = 0;
    s_step_high = false;
    s_step_active = true;
    GPIO.out_w1tc = (1UL << PIN_STEP);

    while (s_step_active) {
        touch_poll_all();
        bool keep = forward ? s_btn_adv.pressed : s_btn_back.pressed;
        if (!keep || s_mode != UI_MAIN) {
            break;
        }

        float t = (float)(esp_timer_get_time() - start_us) / 1000000.0f;
        float cur = fminf(target, fmaxf(MIN_SPEED_MM_S, ACCEL_MM_S2 * t));
        float freq = cur * STEPS_PER_MM;
        if (freq < 1.0f) {
            freq = 1.0f;
        }

        uint32_t period_us = (uint32_t)(1000000.0f / freq);
        if (period_us < 8) {
            period_us = 8;
        }

        if (period_us != last_period_us) {
            if (!motor_timer_set_period(period_us)) {
                break;
            }
            if (last_period_us == 0) {
                if (gptimer_set_raw_count(s_step_timer, 0) != ESP_OK || gptimer_start(s_step_timer) != ESP_OK) {
                    break;
                }
            }
            last_period_us = period_us;
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }

    s_step_active = false;
    gptimer_stop(s_step_timer);
    GPIO.out_w1tc = (1UL << PIN_STEP);
    motor_set_enable(false);
}

static void motor_worker_task(void *arg)
{
    (void)arg;
    while (1) {
        motor_move_cmd_t cmd;
        if (xQueueReceive(s_motor_cmd_queue, &cmd, portMAX_DELAY) == pdTRUE) {
            s_motor_last_cancelled = motor_execute_move(cmd.distance_mm, cmd.target_speed_mm_s, cmd.forward, cmd.with_progress);
            if (s_motor_waiter_task != NULL) {
                xTaskNotifyGive(s_motor_waiter_task);
            }
        }
    }
}

static bool motor_move_mm(float distance_mm, float target_speed_mm_s, bool forward, bool with_progress)
{
    if (s_motor_cmd_queue == NULL) {
        return motor_execute_move(distance_mm, target_speed_mm_s, forward, with_progress);
    }

    motor_move_cmd_t cmd = {
        .distance_mm = distance_mm,
        .target_speed_mm_s = target_speed_mm_s,
        .forward = forward,
        .with_progress = with_progress,
    };

    s_motor_waiter_task = xTaskGetCurrentTaskHandle();
    ulTaskNotifyTake(pdTRUE, 0);
    if (xQueueSend(s_motor_cmd_queue, &cmd, portMAX_DELAY) != pdTRUE) {
        s_motor_waiter_task = NULL;
        return true;
    }
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    s_motor_waiter_task = NULL;
    return s_motor_last_cancelled;
}

static void motor_start_worker(void)
{
    if (s_motor_cmd_queue != NULL) {
        return;
    }
    s_motor_cmd_queue = xQueueCreate(2, sizeof(motor_move_cmd_t));
    if (s_motor_cmd_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create motor queue");
        return;
    }
    if (xTaskCreatePinnedToCore(motor_worker_task, "motor_worker", 4096, NULL, 10, NULL, 1) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create motor worker task on core 1");
    }
}

static void motor_init(void)
{
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << PIN_STEP) | (1ULL << PIN_DIR) | (1ULL << PIN_EN),
        .mode = GPIO_MODE_OUTPUT,
    };
    esp_err_t err = gpio_config(&cfg);
    if (err != ESP_OK) {
        s_motor_ready = false;
        ESP_LOGE(TAG, "Motor pin config failed (%s). Check STEP/DIR/EN pin mapping.", esp_err_to_name(err));
        return;
    }
    s_motor_ready = true;
    gpio_set_level(PIN_STEP, 0);
    gpio_set_level(PIN_DIR, 1);
    motor_set_enable(false);

    gptimer_config_t timer_cfg = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000,
    };
    if (gptimer_new_timer(&timer_cfg, &s_step_timer) != ESP_OK) {
        s_step_timer = NULL;
        s_motor_ready = false;
        ESP_LOGE(TAG, "Failed to create step GPTimer");
        return;
    }

    gptimer_event_callbacks_t cbs = {
        .on_alarm = step_timer_alarm_cb,
    };
    if (gptimer_register_event_callbacks(s_step_timer, &cbs, NULL) != ESP_OK ||
        gptimer_enable(s_step_timer) != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init step GPTimer callbacks");
        s_motor_ready = false;
        return;
    }

    motor_start_worker();
}

static void run_auto_cycle(void)
{
    bool cancelled = false;

    draw_main_screen(s_cycles);
    draw_progress_percent(0.0f);
    cancelled = motor_move_mm(s_cfg.dist_fwd, s_cfg.speed_fwd, true, true);

    if (!cancelled && s_cfg.pause_s > 0.0f) {
        cancelled = wait_with_cancel((uint32_t)(s_cfg.pause_s * 1000.0f));
    }

    if (!cancelled) {
        cancelled = motor_move_mm(s_cfg.dist_back, s_cfg.speed_back, false, true);
    }

    if (!cancelled) {
        s_cycles++;
    }
    draw_main_screen(s_cycles);
}

static void set_edit_buffer_from_menu(void)
{
    switch (s_menu_idx) {
    case MENU_SPEED_FWD:
        snprintf(s_edit_buf, sizeof(s_edit_buf), "%05.2f", s_cfg_edit.speed_fwd);
        break;
    case MENU_DIST_FWD:
        snprintf(s_edit_buf, sizeof(s_edit_buf), "%05.1f", s_cfg_edit.dist_fwd);
        break;
    case MENU_SPEED_BACK:
        snprintf(s_edit_buf, sizeof(s_edit_buf), "%05.2f", s_cfg_edit.speed_back);
        break;
    case MENU_DIST_BACK:
        snprintf(s_edit_buf, sizeof(s_edit_buf), "%05.1f", s_cfg_edit.dist_back);
        break;
    case MENU_PAUSE:
        snprintf(s_edit_buf, sizeof(s_edit_buf), "%04.1f", s_cfg_edit.pause_s);
        break;
    default:
        s_edit_buf[0] = '\0';
        break;
    }
    s_edit_digit_idx = 0;
    while (s_edit_buf[s_edit_digit_idx] == '.') {
        s_edit_digit_idx++;
    }
}

static void apply_edit_buffer_to_cfg(void)
{
    float v = strtof(s_edit_buf, NULL);
    switch (s_menu_idx) {
    case MENU_SPEED_FWD:
        s_cfg_edit.speed_fwd = clampf(v, MIN_SPEED_MM_S, MAX_SPEED_MM_S);
        break;
    case MENU_DIST_FWD:
        s_cfg_edit.dist_fwd = clampf(v, 0.0f, MAX_DISTANCE_MM);
        break;
    case MENU_SPEED_BACK:
        s_cfg_edit.speed_back = clampf(v, MIN_SPEED_MM_S, MAX_SPEED_MM_S);
        break;
    case MENU_DIST_BACK:
        s_cfg_edit.dist_back = clampf(v, 0.0f, MAX_DISTANCE_MM);
        break;
    case MENU_PAUSE:
        s_cfg_edit.pause_s = clampf(v, 0.0f, 60.0f);
        break;
    default:
        break;
    }
}

static void edit_change_digit(int delta)
{
    if (s_edit_buf[s_edit_digit_idx] == '.' || s_edit_buf[s_edit_digit_idx] == '\0') {
        return;
    }
    int d = s_edit_buf[s_edit_digit_idx] - '0';
    d += delta;
    if (d > 9) {
        d = 0;
    }
    if (d < 0) {
        d = 9;
    }
    s_edit_buf[s_edit_digit_idx] = (char)('0' + d);
}

static bool edit_next_digit(void)
{
    int len = (int)strlen(s_edit_buf);
    for (int i = s_edit_digit_idx + 1; i < len; ++i) {
        if (s_edit_buf[i] != '.') {
            s_edit_digit_idx = i;
            return false;
        }
    }
    apply_edit_buffer_to_cfg();
    return true;
}

static void app_loop(void)
{
    draw_boot_screen();
    vTaskDelay(pdMS_TO_TICKS(4000));
    s_mode = UI_MAIN;
    draw_main_screen(s_cycles);
    draw_main_touch_feedback(false, false, false);

    bool last_adv_pressed = false;
    bool last_run_pressed = false;
    bool last_back_pressed = false;

    while (1) {
        touch_poll_all();

        if (s_mode == UI_MAIN) {
            if (s_btn_adv.pressed != last_adv_pressed || s_btn_run.pressed != last_run_pressed || s_btn_back.pressed != last_back_pressed) {
                draw_main_touch_feedback(s_btn_adv.pressed, s_btn_run.pressed, s_btn_back.pressed);
                last_adv_pressed = s_btn_adv.pressed;
                last_run_pressed = s_btn_run.pressed;
                last_back_pressed = s_btn_back.pressed;
            }

            if (s_btn_adv.just_pressed && !s_btn_back.pressed) {
                motor_jog_while_pressed(true, s_cfg.speed_fwd);
                draw_main_touch_feedback(s_btn_adv.pressed, s_btn_run.pressed, s_btn_back.pressed);
                last_adv_pressed = s_btn_adv.pressed;
                last_run_pressed = s_btn_run.pressed;
                last_back_pressed = s_btn_back.pressed;
            } else if (s_btn_back.just_pressed && !s_btn_adv.pressed) {
                motor_jog_while_pressed(false, s_cfg.speed_back);
                draw_main_touch_feedback(s_btn_adv.pressed, s_btn_run.pressed, s_btn_back.pressed);
                last_adv_pressed = s_btn_adv.pressed;
                last_run_pressed = s_btn_run.pressed;
                last_back_pressed = s_btn_back.pressed;
            }

            if (s_btn_run.just_released) {
                uint32_t d = s_btn_run.last_hold_ms;
                if (d >= 100 && d < 2000) {
                    run_auto_cycle();
                    draw_main_touch_feedback(s_btn_adv.pressed, s_btn_run.pressed, s_btn_back.pressed);
                } else if (d >= 4000) {
                    s_cfg_edit = s_cfg;
                    s_menu_idx = MENU_SPEED_FWD;
                    s_mode = UI_CONFIG;
                    draw_config_screen();
                }
            }
        } else if (s_mode == UI_CONFIG) {
            if (s_btn_adv.just_released) {
                s_menu_idx = (menu_item_t)((s_menu_idx + 1) % MENU_COUNT);
                draw_config_screen();
            }
            if (s_btn_back.just_released) {
                s_menu_idx = (menu_item_t)((s_menu_idx + MENU_COUNT - 1) % MENU_COUNT);
                draw_config_screen();
            }
            if (s_btn_run.just_released && s_btn_run.last_hold_ms < 1000) {
                if (s_menu_idx <= MENU_PAUSE) {
                    set_edit_buffer_from_menu();
                    s_mode = UI_EDIT;
                    draw_edit_screen();
                } else if (s_menu_idx == MENU_ABOUT) {
                    s_mode = UI_ABOUT;
                    draw_about_screen();
                } else if (s_menu_idx == MENU_SAVE_EXIT) {
                    s_cfg_edit.magic = MAGIC_CFG;
                    if (cfg_save(&s_cfg_edit) == ESP_OK) {
                        s_cfg = s_cfg_edit;
                    }
                    s_mode = UI_MAIN;
                    draw_main_screen(s_cycles);
                    draw_main_touch_feedback(s_btn_adv.pressed, s_btn_run.pressed, s_btn_back.pressed);
                } else {
                    cfg_load(&s_cfg);
                    s_mode = UI_MAIN;
                    draw_main_screen(s_cycles);
                    draw_main_touch_feedback(s_btn_adv.pressed, s_btn_run.pressed, s_btn_back.pressed);
                }
            }
        } else if (s_mode == UI_EDIT) {
            if (s_btn_adv.just_released) {
                edit_change_digit(+1);
                draw_edit_screen();
            }
            if (s_btn_back.just_released) {
                edit_change_digit(-1);
                draw_edit_screen();
            }
            if (s_btn_run.just_released) {
                if (edit_next_digit()) {
                    s_mode = UI_CONFIG;
                    draw_config_screen();
                } else {
                    draw_edit_screen();
                }
            }
        } else if (s_mode == UI_ABOUT) {
            if (s_btn_run.just_released) {
                s_mode = UI_CONFIG;
                draw_config_screen();
            }
        }

        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

void app_main(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    cfg_load(&s_cfg);
    s_cfg_edit = s_cfg;
    s_cycles = 0;
    s_mode = UI_BOOT;
    s_motor_ready = false;
    s_position_pct = 0.0f;

    ESP_ERROR_CHECK(lcd_init());
    esp_log_level_set("legacy_touch_driver", ESP_LOG_ERROR);
    esp_err_t touch_err = touch_init_and_calibrate();
    if (touch_err != ESP_OK) {
        ESP_LOGE(TAG, "touch init failed: %s", esp_err_to_name(touch_err));
    }
    motor_init();

    app_loop();
}
