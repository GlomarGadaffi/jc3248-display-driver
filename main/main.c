/*
 * SPDX-FileCopyrightText: 2026 GlomarGadaffi
 * SPDX-License-Identifier: Apache-2.0
 *
 * JC3248W535EN feature showcase — a single native-ESP-IDF app that demonstrates
 * everything the board can do: QSPI display, PWM backlight, capacitive touch,
 * microSD, I2S audio, Wi-Fi, and battery sensing. A touch-driven home menu
 * launches each feature demo; each demo returns to the menu via the top bar.
 *
 * THE ONE THING that wastes everyone a day: the init table does NOT issue DISPON,
 * so esp_lcd_panel_disp_on_off(panel, true) is required after init or the panel
 * stays black-but-backlit.
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_touch.h"
#include "esp_lcd_axs15231b.h"
#include "ui.h"
#include "demos.h"
#include "board.h"
#include "usb_msc.h"

static const char *TAG = "jc3248w535";

#define BL_DUTY_FULL 1023   /* full brightness (10-bit) */
#define BL_DUTY_DIM   180   /* dimmed-when-idle level */
#define BL_IDLE_MS  20000   /* dim the backlight after this long with no touch in the menu */

static void backlight_set(int duty)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void backlight_init(void)
{
    ledc_timer_config_t t = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num       = LEDC_TIMER_1,
        .freq_hz         = 5000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&t));
    ledc_channel_config_t c = {
        .gpio_num   = PIN_BL,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_1,
        .intr_type  = LEDC_INTR_DISABLE,
        .duty       = 1023,   /* near-max brightness (10-bit count maxes at 1023) */
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&c));
}

static esp_lcd_panel_handle_t display_init(void)
{
    const spi_bus_config_t buscfg = AXS15231B_PANEL_BUS_QSPI_CONFIG(
        PIN_SCK, PIN_D0, PIN_D1, PIN_D2, PIN_D3,
        LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    esp_lcd_panel_io_handle_t io = NULL;
    /* ui_color_trans_done lets ui_flush wait for each band's DMA to complete. */
    esp_lcd_panel_io_spi_config_t io_cfg = AXS15231B_PANEL_IO_QSPI_CONFIG(PIN_CS, ui_color_trans_done, NULL);
    io_cfg.pclk_hz = 40 * 1000 * 1000;   /* 40 MHz is the stable ceiling on this board.
                                          * 80 MHz runs but is NOT clean: horizontal-line artifacts in
                                          * the image and noise coupled into the I2C touch lines. Don't raise. */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_cfg, &io));

    axs15231b_vendor_config_t vendor = { .flags = { .use_qspi_interface = 1 } };
    esp_lcd_panel_dev_config_t panel_cfg = {
        .reset_gpio_num = -1,
        .rgb_ele_order  = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .vendor_config  = &vendor,
    };
    esp_lcd_panel_handle_t panel = NULL;
    ESP_ERROR_CHECK(esp_lcd_new_panel_axs15231b(io, &panel_cfg, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true)); /* true == display ON */
    return panel;
}

static esp_lcd_touch_handle_t touch_init(void)
{
    i2c_master_bus_handle_t i2c_bus = NULL;
    const i2c_master_bus_config_t i2c_cfg = {
        .i2c_port   = I2C_NUM_0,
        .sda_io_num = PIN_TOUCH_SDA,
        .scl_io_num = PIN_TOUCH_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_cfg, &i2c_bus));

    esp_lcd_panel_io_handle_t tp_io = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_cfg = ESP_LCD_TOUCH_IO_I2C_AXS15231B_CONFIG();
    tp_io_cfg.scl_speed_hz = TOUCH_I2C_HZ;   /* macro omits it; required by the IDF I2C master driver */
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_bus, &tp_io_cfg, &tp_io));

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = LCD_H_RES,
        .y_max = LCD_V_RES,
        .rst_gpio_num = -1,
        .int_gpio_num = -1,
    };
    esp_lcd_touch_handle_t tp = NULL;
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_axs15231b(tp_io, &tp_cfg, &tp));
    return tp;
}

/* ---- Home menu ---- */
static const demo_t DEMOS[] = {
    { "Touch Paint",  demo_touchpaint },
    { "microSD",      demo_sdcard },
    { "Audio",        demo_audio },
    { "WiFi+Battery", demo_wifi_battery },
    { "Animation",    demo_anim },
};
#define NDEMOS (sizeof(DEMOS) / sizeof(DEMOS[0]))

static void draw_menu(ui_t *ui)
{
    const uint16_t cols[NDEMOS] = {
        ui_rgb(200, 40, 40), ui_rgb(40, 160, 40), ui_rgb(40, 80, 200), ui_rgb(200, 140, 30),
        ui_rgb(140, 40, 200),
    };
    int bandh = LCD_V_RES / NDEMOS;
    for (size_t i = 0; i < NDEMOS; i++) {
        ui_rect(ui, 0, (int)i * bandh, LCD_H_RES, bandh, cols[i]);
        /* label, left-padded and vertically centered in the band */
        ui_text(ui, 20, (int)i * bandh + bandh / 2 - 8 * 3 / 2, DEMOS[i].name, 3, ui_rgb(255, 255, 255));
    }
    ui_flush(ui);
}

void app_main(void)
{
    ESP_LOGI(TAG, "JC3248W535EN feature showcase");
    backlight_init();
    esp_lcd_panel_handle_t panel = display_init();

    uint16_t *fb = heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_ERROR_CHECK(fb ? ESP_OK : ESP_ERR_NO_MEM);
    ui_t ui = { .fb = fb, .panel = panel };
    ui_init(&ui);   /* internal-RAM bounce buffer + flush sync */

    /* One-shot: if the SD demo asked for it, come up as a USB drive instead of the
     * app (touch/Wi-Fi/etc. are skipped). Never returns; unplug USB to go back. */
    if (usb_msc_requested()) {
        usb_msc_run(&ui);
    }

    esp_lcd_touch_handle_t tp = touch_init();

    draw_menu(&ui);
    ESP_LOGI(TAG, "home menu ready — tap a band to launch a demo (top->bottom):");
    for (size_t i = 0; i < NDEMOS; i++) ESP_LOGI(TAG, "  [%u] %s", (unsigned)i, DEMOS[i].name);

    int bandh = LCD_V_RES / NDEMOS;
    uint16_t x = 0, y = 0;
    bool prev = false;
    int idle_ms = 0;
    bool dimmed = false;
    while (1) {
        bool pressed = board_touch(tp, &x, &y);
        if (pressed) {
            idle_ms = 0;
            if (dimmed) { backlight_set(BL_DUTY_FULL); dimmed = false; }
        }
        if (pressed && !prev) {
            int idx = y / bandh;
            if (idx >= 0 && idx < (int)NDEMOS) {
                ESP_LOGI(TAG, "launch: %s", DEMOS[idx].name);
                DEMOS[idx].run(&ui, tp);            /* blocks until the demo returns */
                backlight_set(BL_DUTY_FULL);        /* a demo may have run a while; wake the panel */
                dimmed = false; idle_ms = 0;
                draw_menu(&ui);                     /* redraw the menu on return */
                while (board_touch(tp, &x, &y))     /* swallow the release */
                    vTaskDelay(pdMS_TO_TICKS(20));
            }
        }
        prev = pressed;
        vTaskDelay(pdMS_TO_TICKS(30));
        idle_ms += 30;
        if (!dimmed && idle_ms >= BL_IDLE_MS) { backlight_set(BL_DUTY_DIM); dimmed = true; }
    }
}
