/*
 * SPDX-FileCopyrightText: 2026 GlomarGadaffi
 * SPDX-License-Identifier: Apache-2.0
 *
 * Feature-demo interface. Each demo takes over the screen and runs until the
 * user taps the top "back" bar (ui_in_back), then returns to the home menu.
 * One demo per source file so they can be developed independently.
 */
#pragma once

#include "ui.h"

typedef struct {
    const char *name;
    void (*run)(ui_t *ui, esp_lcd_touch_handle_t tp);
} demo_t;

void demo_touchpaint(ui_t *ui, esp_lcd_touch_handle_t tp);
void demo_sdcard(ui_t *ui, esp_lcd_touch_handle_t tp);
void demo_audio(ui_t *ui, esp_lcd_touch_handle_t tp);
void demo_wifi_battery(ui_t *ui, esp_lcd_touch_handle_t tp);
void demo_anim(ui_t *ui, esp_lcd_touch_handle_t tp);
