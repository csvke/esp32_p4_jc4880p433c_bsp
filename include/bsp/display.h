#pragma once
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "hal/lcd_types.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    lvgl_port_cfg_t lvgl_port_cfg;
    uint32_t        buffer_size;
    bool            double_buffer;
    struct {
        unsigned int buff_dma: 1;
        unsigned int buff_spiram: 1;
        unsigned int sw_rotate: 1;
    } flags;
} bsp_display_cfg_t;

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);
bool bsp_display_lock(uint32_t timeout_ms);
void bsp_display_unlock(void);
esp_err_t bsp_display_backlight_on(void);

#ifdef __cplusplus
}
#endif

