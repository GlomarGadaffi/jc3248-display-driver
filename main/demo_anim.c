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
static volatile uint32_t s_late;

/* Start at most one frame per edge. An unused edge that fired under LATE_OK_US ago still
 * counts: starting that late keeps the writer behind the scan, so it stays tear-free,
 * and waiting a whole period for the next edge would halve the frame rate instead. */
#define LATE_OK_US 2000
/* ponytail: 60 fps holds with ~0.3 ms to spare. On COM8: flush max 15.88 ms vs TE period
 * 16.19 ms, one unit, 40 MHz QSPI (the S3 GPSPI clock is 80/(pre*n) with n >= 2, so nothing
 * exists between 40 and 80; see spi_ll_master_cal_clock). A panel with a shorter TE period
 * shows periodic hitches instead of a clean 30: each frame starts a bit later until the
 * start falls outside LATE_OK_US. Upgrade path: fewer bands (UI_BAND_ROWS 96/120 if internal
 * RAM allows), or one CS-held RAMWR stream via the raw spi_device API, which removes the
 * blocking command per band that esp_lcd's tx_color adds. */
static void wait_te(uint32_t *used)
{
    if (s_te_edges == *used || esp_timer_get_time() - s_te_fall > LATE_OK_US) {
        xSemaphoreTake(s_te, 0);                          /* drop a stale edge */
        if (xSemaphoreTake(s_te, pdMS_TO_TICKS(50)) != pdTRUE) s_no_te++;
    } else {
        xSemaphoreTake(s_te, 0);
        s_late++;
    }
    *used = s_te_edges;
}

/* Streams frames back to back. While a frame's last band is on the bus, the next frame's
 * band 0 is staged into the other bounce buffer, so the only gap between frames is the
 * wait for the next TE edge. UI_BANDS is even, so the last band and band 0 never share
 * a bounce buffer. */
_Static_assert(UI_BANDS % 2 == 0, "last band and next band 0 must use different bounce buffers");

static void flush_task(void *arg)
{
    esp_lcd_panel_handle_t panel = arg;
    uint16_t *buf, *next;
    uint32_t used = 0;
    if (xQueueReceive(s_ready, &buf, portMAX_DELAY) != pdTRUE) buf = NULL;
    if (buf) ui_band_stage(buf, 0);
    while (buf) {
        wait_te(&used);
        int64_t t0 = esp_timer_get_time();
        int inflight = 0;
        for (int band = 0; band < UI_BANDS; band++) {
            if (band > 0) {
                if (inflight == 2) { ui_band_wait(); inflight--; }
                ui_band_stage(buf, band);
            }
            if (ui_band_send(panel, band)) inflight++;
        }
        xQueueSend(s_free, &buf, portMAX_DELAY);          /* every band is copied out: render may reuse it */
        if (inflight == 2) { ui_band_wait(); inflight--; } /* frees band 0's bounce buffer */
        xQueueReceive(s_ready, &next, portMAX_DELAY);
        if (next) ui_band_stage(next, 0);                  /* overlaps the last band's DMA */
        while (inflight > 0) { ui_band_wait(); inflight--; }
        int64_t d = esp_timer_get_time() - t0;
        if (s_te_edges != used) s_overruns++;             /* the next scan began mid-flush */
        s_flush_sum += d;
        if (d > s_flush_max) s_flush_max = d;
        s_frames++;
        buf = next;
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
    s_frames = s_overruns = s_no_te = s_te_edges = s_late = 0;
    s_flush_sum = s_flush_max = 0;
    s_te_fall = s_te_rise = s_te_period = s_te_low = s_te_high = 0;

    const gpio_config_t te_cfg = {
        .pin_bit_mask = 1ULL << PIN_TE, .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, .intr_type = GPIO_INTR_ANYEDGE,
    };
    gpio_config(&te_cfg);
    static bool isr_service;   /* IDF logs an E line if it's installed twice; once per boot */
    if (!isr_service) {
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(err);
        isr_service = true;
    }
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
    int64_t last = esp_timer_get_time(), stat_t = last, render_sum = 0, stat_flush = 0;
    uint32_t stat_frames = 0, stat_ovr = 0, stat_note = 0, stat_late = 0, renders = 0;
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

        if (now - stat_t >= 1000000) {   /* per-window stats: each line stands on its own */
            uint32_t f = s_frames, df = f - stat_frames, ovr = s_overruns - stat_ovr, note = s_no_te - stat_note;
            uint32_t late = s_late - stat_late;
            int64_t fsum = s_flush_sum;
            float fps = df * 1e6f / (now - stat_t);
            float flush_avg = df ? (fsum - stat_flush) / 1000.0f / df : 0;
            float flush_max = s_flush_max / 1000.0f;
            s_flush_max = 0;
            snprintf(hud1, sizeof hud1, "%.1f FPS", fps);
            snprintf(hud2, sizeof hud2, "flush %.1fms TE %.1fms ovr %lu/s",
                     flush_avg, s_te_period / 1000.0f, (unsigned long)ovr);
            ESP_LOGI(TAG, "%.1f fps | flush avg %.2f max %.2f ms | render avg %.2f ms | "
                     "TE period %.2f low %.2f high %.2f ms | overruns %lu/%lu frames | late starts %lu | no-TE %lu",
                     fps, flush_avg, flush_max, render_sum / 1000.0f / renders,
                     s_te_period / 1000.0f, s_te_low / 1000.0f, s_te_high / 1000.0f,
                     (unsigned long)ovr, (unsigned long)df, (unsigned long)late, (unsigned long)note);
            stat_t = now;
            stat_frames = f;
            stat_flush = fsum;
            stat_ovr += ovr;
            stat_note += note;
            stat_late += late;
            render_sum = 0;
            renders = 0;
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
