#pragma once

#include "sdkconfig.h"
#include "bsp/config.h"
#include "bsp/display.h"
#include "esp_lcd_touch.h"

#ifdef __cplusplus
extern "C" {
#endif

// Display configuration (configurable via menuconfig)
#define BSP_LCD_H_RES                   CONFIG_BSP_JC4880P443C_LCD_H_RES
#define BSP_LCD_V_RES                   CONFIG_BSP_JC4880P443C_LCD_V_RES
#define BSP_LCD_COLOR_SPACE             LCD_RGB_ELEMENT_ORDER_RGB

// GPIO pin definitions (configurable via menuconfig)
#define BSP_I2C_SCL                     ((gpio_num_t)CONFIG_BSP_JC4880P443C_I2C_SCL_GPIO)
#define BSP_I2C_SDA                     ((gpio_num_t)CONFIG_BSP_JC4880P443C_I2C_SDA_GPIO)
#define BSP_LCD_BACKLIGHT               ((gpio_num_t)CONFIG_BSP_JC4880P443C_LCD_BL_GPIO)
#define BSP_LCD_RST                     ((gpio_num_t)CONFIG_BSP_JC4880P443C_LCD_RST_GPIO)

// Audio pins (not configurable - board specific)
#define BSP_I2S_SCLK                    (GPIO_NUM_12)
#define BSP_I2S_MCLK                    (GPIO_NUM_13)
#define BSP_I2S_LCLK                    (GPIO_NUM_10)
#define BSP_I2S_DOUT                    (GPIO_NUM_9)
#define BSP_I2S_DSIN                    (GPIO_NUM_48)
#define BSP_POWER_AMP_IO                (GPIO_NUM_11)

// Touch pins (not connected to dedicated pins on this board)
#define BSP_LCD_TOUCH_RST               (GPIO_NUM_NC)
#define BSP_LCD_TOUCH_INT               (GPIO_NUM_NC)

// SD Pins are optional on this board; leave unconfigured for now

// API surface compatible with esp32_p4_function_ev_board BSP (subset)
esp_err_t bsp_spiffs_mount(void);
esp_err_t bsp_sdcard_mount(void);
esp_err_t bsp_display_backlight_on(void);
esp_err_t bsp_display_brightness_set(uint8_t brightness_percent);

// Display API
bsp_display_cfg_t bsp_display_get_default_config(void);
lv_display_t *bsp_display_start(void);
lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);
bool bsp_display_lock(uint32_t timeout_ms);
void bsp_display_unlock(void);

// Touch helper
esp_err_t bsp_touch_new(const void *config, esp_lcd_touch_handle_t *ret_touch);

// Audio extra API (stubbed for now)
esp_err_t bsp_extra_codec_init();

#ifdef __cplusplus
}
#endif
