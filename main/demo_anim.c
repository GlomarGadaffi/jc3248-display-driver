/*
 * SPDX-FileCopyrightText: 2026 GlomarGadaffi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Animation demo: tear-free, TE-paced full-frame animation, and a measurement of it.
 *
 * The AXS15231B QSPI path only takes full-screen top-to-bottom writes, but that does
 * not mean every frame has to be fully *re-rendered*. Each frame here redraws only
 * what moved (dirty rects) into one of two PSRAM framebuffers, while a flush task on
 * the other core streams the previous frame to the panel. Each flush starts on the
 * TE falling edge: the panel has just begun scanning GRAM top to bottom, so a writer
 * that is slower than the scan stays behind it and never shows a half-new frame.
 * The window is roughly Tvdl < flush < period + Tvdl, and a flush that fits inside
 * one TE period gives a locked 60 fps.
 *
 * The serial log prints fps, flush time, TE period/widths and overruns once a second;
 * those numbers decide whether 60 fps holds or the clock / start edge needs tuning.
 */
#include <stdio.h>
#include <string.h>
#include "demos.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

static const char *TAG = "demo.anim";

#define TE_SYNC_RISING 0   /* 0: start on the falling edge (scan start, the vendor BSP's choice);
                            * 1: start on the rising edge (blanking start, pocket-dial's choice) */
#define HUD_Y   UI_BACK_H
#define HUD_H   32
#define PLAY_Y  (HUD_Y + HUD_H)
#define TICK_H  40
#define TICK_Y  (LCD_V_RES - TICK_H)
#define N_BALLS 10
#define BALL_R  14
#define BAR_W   8
#define MAX_DIRTY (N_BALLS + 1)

/* ---- TE capture ---- */
static SemaphoreHandle_t s_te;
static volatile int64_t s_te_fall, s_te_rise, s_te_period, s_te_low, s_te_high;
static volatile uint32_t s_te_edges;

static void IRAM_ATTR te_isr(void *arg)
{
    (void)arg;
    int64_t now = esp_timer_get_time();
    bool rising = gpio_get_level(PIN_TE);
    if (rising) {                   /* scan done, blanking starts */
        s_te_low = now - s_te_fall;
        s_te_rise = now;
    } else {                        /* panel starts scanning GRAM top to bottom */
        s_te_period = now - s_te_fall;
        s_te_high = now - s_te_rise;
        s_te_fall = now;
    }
    if (rising == TE_SYNC_RISING) {
        s_te_edges++;
        BaseType_t hp = pdFALSE;
        xSemaphoreGiveFromISR(s_te, &hp);
        if (hp) portYIELD_FROM_ISR();
    }
}

/* ---- flush task: one frame per TE edge ---- */
static QueueHandle_t s_ready, s_free;
static SemaphoreHandle_t s_exit;
static volatile uint32_t s_frames, s_overruns, s_no_te;
static volatile int64_t s_flush_sum, s_flush_max;

static void flush_task(void *arg)
{
    esp_lcd_panel_handle_t panel = arg;
    uint16_t *buf;
    while (xQueueReceive(s_ready, &buf, portMAX_DELAY) == pdTRUE && buf) {
        xSemaphoreTake(s_te, 0);                          /* drop a stale edge */
        if (xSemaphoreTake(s_te, pdMS_TO_TICKS(50)) != pdTRUE) s_no_te++;
        uint32_t edge = s_te_edges;
        int64_t t0 = esp_timer_get_time();
        ui_flush(&(ui_t){ .fb = buf, .panel = panel });
        int64_t d = esp_timer_get_time() - t0;
        if (s_te_edges != edge) s_overruns++;             /* the next scan began mid-flush */
        s_flush_sum += d;
        if (d > s_flush_max) s_flush_max = d;
        s_frames++;
        xQueueSend(s_free, &buf, portMAX_DELAY);
    }
    xSemaphoreGive(s_exit);
    vTaskDelete(NULL);
}

/* ---- scene ---- */
typedef struct { int x, y, w, h; } rect_t;
typedef struct { float x, y, vx, vy; uint16_t color; } ball_t;

static void ball_fill(ui_t *ui, int cx, int cy, int r, uint16_t c)
{
    for (int dy = -r; dy <= r; dy++) {
        int dx = 0;
        while ((dx + 1) * (dx + 1) + dy * dy <= r * r) dx++;
        ui_rect(ui, cx - dx, cy + dy, 2 * dx + 1, 1, c);
    }
}

void demo_anim(ui_t *ui, esp_lcd_touch_handle_t tp)
{
    ESP_LOGI(TAG, "animation demo — tap the back bar to exit");

    uint16_t *bufs[2] = { ui->fb, heap_caps_malloc(LCD_H_RES * LCD_V_RES * sizeof(uint16_t),
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) };
    if (!bufs[1]) { ESP_LOGE(TAG, "no PSRAM for the second framebuffer"); return; }

    s_te = xSemaphoreCreateBinary();
    s_exit = xSemaphoreCreateBinary();
    s_ready = xQueueCreate(2, sizeof(uint16_t *));
    s_free = xQueueCreate(2, sizeof(uint16_t *));
    s_frames = s_overruns = s_no_te = 0;
    s_flush_sum = s_flush_max = 0;

    const gpio_config_t te_cfg = {
        .pin_bit_mask = 1ULL << PIN_TE, .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&te_cfg);
    esp_err_t err = gpio_install_isr_service(0);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
    gpio_isr_handler_add(PIN_TE, te_isr, NULL);

    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "TE: %lu sync edges in 200 ms, period %lld us, low %lld us, high %lld us",
             (unsigned long)s_te_edges, s_te_period, s_te_low, s_te_high);
    if (s_te_edges == 0) ESP_LOGW(TAG, "no TE edges on GPIO%d: frames will be unpaced", PIN_TE);

    /* Static parts go into both buffers once; everything else is dirty-rect redrawn. */
    for (int i = 0; i < 2; i++) {
        ui_t u = { .fb = bufs[i], .panel = ui->panel };
        memset(bufs[i], 0, LCD_H_RES * LCD_V_RES * sizeof(uint16_t));
        ui_back_bar(&u);
        ui_text(&u, 150, 16, "ANIM", 2, ui_rgb(255, 255, 255));
        xQueueSend(s_free, &bufs[i], 0);
    }
    xTaskCreatePinnedToCore(flush_task, "anim_flush", 3072, ui->panel, 5, NULL, 1);

    ball_t balls[N_BALLS];
    for (int i = 0; i < N_BALLS; i++) {
        balls[i] = (ball_t){
            .x = 30 + i * 27, .y = PLAY_Y + 30 + (i * 37) % 300,
            .vx = (i & 1 ? 1 : -1) * (90 + i * 23), .vy = (i & 2 ? 1 : -1) * (120 + i * 17),
            .color = ui_rgb(80 + i * 17, 255 - i * 20, 60 + i * 19),
        };
    }
    rect_t dirty[2][MAX_DIRTY];
    int ndirty[2] = { 0, 0 };
    float bar_x = 0, tick = 0;
    char hud1[24] = "", hud2[48] = "";
    int64_t last = esp_timer_get_time(), stat_t = last, render_sum = 0;
    uint32_t stat_frames = 0, renders = 0;
    const uint16_t black = 0, bar_c = ui_rgb(255, 255, 255);
    uint16_t x, y;

    while (!(board_touch(tp, &x, &y) && ui_in_back(x, y))) {
        uint16_t *buf;
        xQueueReceive(s_free, &buf, portMAX_DELAY);
        int b = buf == bufs[1];
        ui_t u = { .fb = buf, .panel = ui->panel };
        int64_t now = esp_timer_get_time();
        float dt = (now - last) / 1e6f;
        last = now;

        for (int i = 0; i < ndirty[b]; i++)
            ui_rect(&u, dirty[b][i].x, dirty[b][i].y, dirty[b][i].w, dirty[b][i].h, black);
        ndirty[b] = 0;

        /* full-height bar sweeping sideways: a tear shows up as a step in it */
        bar_x += 300 * dt;
        if (bar_x > LCD_H_RES - BAR_W) bar_x -= LCD_H_RES - BAR_W;
        rect_t bar = { (int)bar_x, PLAY_Y, BAR_W, TICK_Y - PLAY_Y };
        ui_rect(&u, bar.x, bar.y, bar.w, bar.h, bar_c);
        dirty[b][ndirty[b]++] = bar;

        for (int i = 0; i < N_BALLS; i++) {
            ball_t *p = &balls[i];
            p->x += p->vx * dt;
            p->y += p->vy * dt;
            if (p->x < BALL_R) { p->x = BALL_R; p->vx = -p->vx; }
            if (p->x > LCD_H_RES - 1 - BALL_R) { p->x = LCD_H_RES - 1 - BALL_R; p->vx = -p->vx; }
            if (p->y < PLAY_Y + BALL_R) { p->y = PLAY_Y + BALL_R; p->vy = -p->vy; }
            if (p->y > TICK_Y - 1 - BALL_R) { p->y = TICK_Y - 1 - BALL_R; p->vy = -p->vy; }
            ball_fill(&u, (int)p->x, (int)p->y, BALL_R, p->color);
            dirty[b][ndirty[b]++] = (rect_t){ (int)p->x - BALL_R, (int)p->y - BALL_R,
                                              2 * BALL_R + 1, 2 * BALL_R + 1 };
        }

        /* full-width scrolling stripes */
        tick += 240 * dt;
        int off = (int)tick % 40;
        for (int sx = -off; sx < LCD_H_RES; sx += 20)
            ui_rect(&u, sx, TICK_Y, 20, TICK_H, ((sx + off) / 20) & 1 ? ui_rgb(255, 120, 0) : ui_rgb(0, 90, 255));

        ui_rect(&u, 0, HUD_Y, LCD_H_RES, HUD_H, ui_rgb(20, 20, 20));
        ui_text(&u, 8, HUD_Y + 2, hud1, 2, ui_rgb(0, 255, 120));
        ui_text(&u, 8, HUD_Y + 21, hud2, 1, ui_rgb(200, 200, 200));

        render_sum += esp_timer_get_time() - now;
        renders++;
        xQueueSend(s_ready, &buf, portMAX_DELAY);

        if (now - stat_t >= 1000000) {
            uint32_t f = s_frames;
            float fps = (f - stat_frames) * 1e6f / (now - stat_t);
            float flush_avg = f ? s_flush_sum / 1000.0f / f : 0;
            snprintf(hud1, sizeof hud1, "%.1f FPS", fps);
            snprintf(hud2, sizeof hud2, "flush %.1fms TE %.1fms ovr %lu",
                     flush_avg, s_te_period / 1000.0f, (unsigned long)s_overruns);
            ESP_LOGI(TAG, "%.1f fps | flush avg %.2f max %.2f ms | render avg %.2f ms | "
                     "TE period %.2f low %.2f high %.2f ms | overruns %lu | no-TE %lu",
                     fps, flush_avg, s_flush_max / 1000.0f, render_sum / 1000.0f / renders,
                     s_te_period / 1000.0f, s_te_low / 1000.0f, s_te_high / 1000.0f,
                     (unsigned long)s_overruns, (unsigned long)s_no_te);
            stat_t = now;
            stat_frames = f;
        }
    }

    uint16_t *stop = NULL;
    xQueueSend(s_ready, &stop, portMAX_DELAY);
    xSemaphoreTake(s_exit, portMAX_DELAY);
    gpio_isr_handler_remove(PIN_TE);
    gpio_set_intr_type(PIN_TE, GPIO_INTR_DISABLE);
    vQueueDelete(s_ready);
    vQueueDelete(s_free);
    vSemaphoreDelete(s_te);
    vSemaphoreDelete(s_exit);
    heap_caps_free(bufs[1]);
    ESP_LOGI(TAG, "exit: %lu frames, %lu overruns", (unsigned long)s_frames, (unsigned long)s_overruns);
}
