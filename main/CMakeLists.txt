idf_component_register(
    SRCS
        "main.c"
        "ui.c"
        "demo_touchpaint.c"
        "demo_sdcard.c"
        "demo_audio.c"
        "demo_wifi_battery.c"
        "demo_anim.c"
        "usb_msc.c"
    INCLUDE_DIRS "."
    REQUIRES
        esp_lcd_axs15231b esp_lcd esp_lcd_touch
        driver esp_driver_ledc esp_driver_gpio esp_driver_spi esp_driver_i2c
        esp_driver_i2s esp_driver_sdmmc fatfs vfs
        esp_wifi esp_adc nvs_flash esp_netif esp_event esp_timer
        esp_tinyusb
)
