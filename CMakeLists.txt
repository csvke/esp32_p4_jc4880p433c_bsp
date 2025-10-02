idf_component_register(
    SRCS "src/jc4880p4_bsp.c"
    INCLUDE_DIRS "include"
    PRIV_REQUIRES spiffs esp_lcd esp_lcd_st7701 esp_lcd_touch esp_lcd_touch_gt911 esp_lvgl_port esp_driver_i2c esp_driver_ledc esp_hw_support
)
