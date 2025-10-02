#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"
#include "esp_spiffs.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_st7701.h"
#include "esp_lcd_touch_gt911.h"
#include "esp_ldo_regulator.h"
#include "driver/ledc.h"
#include "driver/i2c_master.h"
#include "bsp/display.h"
#include "bsp/esp-bsp.h"

static const char *TAG = "JC4880P4_BSP";
static i2c_master_bus_handle_t i2c_handle = NULL;

// ST7701 vendor initialization sequence for 480x800 panel
static const st7701_lcd_init_cmd_t st7701_lcd_cmds[] = {
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x13}, 5, 0},
    {0xEF, (uint8_t[]){0x08}, 1, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x10}, 5, 0},
    {0xC0, (uint8_t[]){0x63, 0x00}, 2, 0},
    {0xC1, (uint8_t[]){0x0D, 0x02}, 2, 0},
    {0xC2, (uint8_t[]){0x10, 0x08}, 2, 0},
    {0xCC, (uint8_t[]){0x10}, 1, 0},

    {0xB0, (uint8_t[]){0x80, 0x09, 0x53, 0x0C, 0xD0, 0x07, 0x0C, 0x09, 0x09, 0x28, 0x06, 0xD4, 0x13, 0x69, 0x2B, 0x71}, 16, 0},
    {0xB1, (uint8_t[]){0x80, 0x94, 0x5A, 0x10, 0xD3, 0x06, 0x0A, 0x08, 0x08, 0x25, 0x03, 0xD3, 0x12, 0x66, 0x6A, 0x0D}, 16, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x11}, 5, 0},

    {0xB0, (uint8_t[]){0x5D}, 1, 0},
    {0xB1, (uint8_t[]){0x58}, 1, 0},
    {0xB2, (uint8_t[]){0x87}, 1, 0},
    {0xB3, (uint8_t[]){0x80}, 1, 0},
    {0xB5, (uint8_t[]){0x4E}, 1, 0},
    {0xB7, (uint8_t[]){0x85}, 1, 0},
    {0xB8, (uint8_t[]){0x21}, 1, 0},
    {0xB9, (uint8_t[]){0x10, 0x1F}, 2, 0},
    {0xBB, (uint8_t[]){0x03}, 1, 0},
    {0xBC, (uint8_t[]){0x00}, 1, 0},

    {0xC1, (uint8_t[]){0x78}, 1, 0},
    {0xC2, (uint8_t[]){0x78}, 1, 0},
    {0xD0, (uint8_t[]){0x88}, 1, 0},

    {0xE0, (uint8_t[]){0x00, 0x3A, 0x02}, 3, 0},
    {0xE1, (uint8_t[]){0x04, 0xA0, 0x00, 0xA0, 0x05, 0xA0, 0x00, 0xA0, 0x00, 0x40, 0x40}, 11, 0},
    {0xE2, (uint8_t[]){0x30, 0x00, 0x40, 0x40, 0x32, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00, 0xA0, 0x00}, 13, 0},
    {0xE3, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE4, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE5, (uint8_t[]){0x09, 0x2E, 0xA0, 0xA0, 0x0B, 0x30, 0xA0, 0xA0, 0x05, 0x2A, 0xA0, 0xA0, 0x07, 0x2C, 0xA0, 0xA0}, 16, 0},
    {0xE6, (uint8_t[]){0x00, 0x00, 0x33, 0x33}, 4, 0},
    {0xE7, (uint8_t[]){0x44, 0x44}, 2, 0},
    {0xE8, (uint8_t[]){0x08, 0x2D, 0xA0, 0xA0, 0x0A, 0x2F, 0xA0, 0xA0, 0x04, 0x29, 0xA0, 0xA0, 0x06, 0x2B, 0xA0, 0xA0}, 16, 0},

    {0xEB, (uint8_t[]){0x00, 0x00, 0x4E, 0x4E, 0x00, 0x00, 0x00}, 7, 0},
    {0xEC, (uint8_t[]){0x08, 0x01}, 2, 0},

    {0xED, (uint8_t[]){0xB0, 0x2B, 0x98, 0xA4, 0x56, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF, 0xF7, 0x65, 0x4A, 0x89, 0xB2, 0x0B}, 16, 0},
    {0xEF, (uint8_t[]){0x08, 0x08, 0x08, 0x45, 0x3F, 0x54}, 6, 0},
    {0xFF, (uint8_t[]){0x77, 0x01, 0x00, 0x00, 0x00}, 5, 0},

    {0x11, (uint8_t[]){0x00}, 1, 120},
    {0x29, (uint8_t[]){0x00}, 1, 20},
};

static esp_err_t bsp_i2c_init(void)
{
    if (i2c_handle) return ESP_OK;
    i2c_master_bus_config_t i2c_bus_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .sda_io_num = BSP_I2C_SDA,
        .scl_io_num = BSP_I2C_SCL,
        .i2c_port = 1,
    };
    return i2c_new_master_bus(&i2c_bus_conf, &i2c_handle);
}

esp_err_t bsp_spiffs_mount(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "storage",
        .max_files = 5,
        .format_if_mount_failed = false,
    };
    ESP_RETURN_ON_ERROR(esp_vfs_spiffs_register(&conf), TAG, "spiffs mount failed");
    size_t total=0, used=0;
    esp_spiffs_info(conf.partition_label, &total, &used);
    ESP_LOGI(TAG, "SPIFFS total=%zu used=%zu", total, used);
    return ESP_OK;
}

esp_err_t bsp_sdcard_mount(void)
{
    // Optional: left unimplemented for now
    return ESP_OK;
}

// Power up the MIPI DSI PHY using internal LDO (VO3) at 2.5V
static esp_err_t bsp_enable_dsi_phy_power(void)
{
    static esp_ldo_channel_handle_t phy_pwr_chan = NULL;
    if (phy_pwr_chan) {
        return ESP_OK;
    }
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id = CONFIG_BSP_JC4880P443C_DSI_PHY_LDO_CHANNEL,
        .voltage_mv = CONFIG_BSP_JC4880P443C_DSI_PHY_VOLTAGE_MV,
    };
    esp_err_t err = esp_ldo_acquire_channel(&ldo_cfg, &phy_pwr_chan);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MIPI DSI PHY power enabled (LDO ch%d: %dmV)", ldo_cfg.chan_id, ldo_cfg.voltage_mv);
    } else {
        ESP_LOGW(TAG, "Failed to enable MIPI DSI PHY power: %s", esp_err_to_name(err));
    }
    return err;
}

static esp_err_t bsp_display_brightness_init(void)
{
    const ledc_channel_config_t ch = {
        .gpio_num = CONFIG_BSP_JC4880P443C_LCD_BL_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = CONFIG_BSP_JC4880P443C_BACKLIGHT_CHANNEL,
        .duty = 0,
        .timer_sel = CONFIG_BSP_JC4880P443C_BACKLIGHT_TIMER
    };
    const ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = CONFIG_BSP_JC4880P443C_BACKLIGHT_TIMER,
        .freq_hz = CONFIG_BSP_JC4880P443C_BACKLIGHT_PWM_FREQ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&t), TAG, "ledc timer");
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "ledc ch");
    return ESP_OK;
}

esp_err_t bsp_display_backlight_on(void)
{
    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "bl init");
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, CONFIG_BSP_JC4880P443C_BACKLIGHT_CHANNEL, 1023), TAG, "set duty");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, CONFIG_BSP_JC4880P443C_BACKLIGHT_CHANNEL);
}

esp_err_t bsp_display_brightness_set(uint8_t brightness_percent)
{
    if (brightness_percent > 100) {
        return ESP_ERR_INVALID_ARG;
    }
    
    ESP_RETURN_ON_ERROR(bsp_display_brightness_init(), TAG, "bl init");
    
    // Convert percentage to duty cycle (0-1023 for 10-bit resolution)
    uint32_t duty = (brightness_percent * 1023) / 100;
    ESP_RETURN_ON_ERROR(ledc_set_duty(LEDC_LOW_SPEED_MODE, CONFIG_BSP_JC4880P443C_BACKLIGHT_CHANNEL, duty), TAG, "set duty");
    return ledc_update_duty(LEDC_LOW_SPEED_MODE, CONFIG_BSP_JC4880P443C_BACKLIGHT_CHANNEL);
}

bsp_display_cfg_t bsp_display_get_default_config(void)
{
    bsp_display_cfg_t cfg = {
        .lvgl_port_cfg = ESP_LVGL_PORT_INIT_CONFIG(),
        .buffer_size = CONFIG_BSP_JC4880P443C_LVGL_BUFFER_SIZE,
        .double_buffer = CONFIG_BSP_JC4880P443C_LVGL_DOUBLE_BUFFER,
        .flags = {
            .buff_dma = true,
            .buff_spiram = true,
            .sw_rotate = false,
        }
    };
    return cfg;
}

lv_display_t *bsp_display_start(void)
{
    bsp_display_cfg_t cfg = bsp_display_get_default_config();
    return bsp_display_start_with_config(&cfg);
}

lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg)
{
    ESP_LOGI(TAG, "Init display");

    // DSI bus and DBI IO for ST7701
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = {
        .bus_id = 0,
        .num_data_lanes = 2,
        .phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT,
        .lane_bit_rate_mbps = CONFIG_BSP_JC4880P443C_DSI_LANE_BITRATE,
    };
    // Ensure MIPI DPHY is powered before creating the bus
    (void)bsp_enable_dsi_phy_power();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));
    // Give PHY a moment to stabilize before DBI transfers
    vTaskDelay(pdMS_TO_TICKS(50));

    esp_lcd_panel_io_handle_t io;
    esp_lcd_dbi_io_config_t dbi_config = {
        .virtual_channel = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io));

    esp_lcd_panel_handle_t panel = NULL;
    // Explicit DPI timing for 480x800 panel at ~60Hz (vendor reference)
    esp_lcd_dpi_panel_config_t dpi_config = {
        .dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT,
        .dpi_clock_freq_mhz = CONFIG_BSP_JC4880P443C_DPI_CLOCK_MHZ,
        .virtual_channel = 0,
        .pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565,
        .num_fbs = CONFIG_BSP_JC4880P443C_NUM_FB,
        .video_timing = {
            .h_size = CONFIG_BSP_JC4880P443C_LCD_H_RES,
            .v_size = CONFIG_BSP_JC4880P443C_LCD_V_RES,
            .hsync_pulse_width = 12,
            .hsync_back_porch = 42,
            .hsync_front_porch = 42,
            .vsync_pulse_width = 2,
            .vsync_back_porch = 8,
            .vsync_front_porch = 166,
        },
        .flags = { .use_dma2d = true },
    };

    st7701_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
        .init_cmds = st7701_lcd_cmds,
        .init_cmds_size = sizeof(st7701_lcd_cmds) / sizeof(st7701_lcd_cmds[0]),
        .flags = { .use_mipi_interface = 1 }
    };
    esp_lcd_panel_dev_config_t lcd_dev_config = {
        .bits_per_pixel = 16,
        .rgb_ele_order = BSP_LCD_COLOR_SPACE,
        .reset_gpio_num = BSP_LCD_RST,
        .vendor_config = &vendor_config,
    };

    ESP_ERROR_CHECK(esp_lcd_new_panel_st7701(io, &lcd_dev_config, &panel));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel));
    // Ensure panel display is turned on (some sequences rely on explicit ON)
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));

    // LVGL port
    lvgl_port_cfg_t port_cfg = cfg->lvgl_port_cfg;
    ESP_ERROR_CHECK(lvgl_port_init(&port_cfg));

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io,
        .panel_handle = panel,
        .control_handle = NULL,
        .buffer_size = cfg->buffer_size,
        .double_buffer = cfg->double_buffer,
        .hres = BSP_LCD_H_RES,
        .vres = BSP_LCD_V_RES,
        .monochrome = false,
    // Portrait 480x800 timing, no XY swap
    .rotation = { .swap_xy = false, .mirror_x = false, .mirror_y = false },
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .flags = { .buff_dma = cfg->flags.buff_dma, .buff_spiram = cfg->flags.buff_spiram, .sw_rotate = cfg->flags.sw_rotate }
    };
    // Disable avoid_tearing initially to prevent blocking on VSYNC during bring-up
    const lvgl_port_display_dsi_cfg_t dsi_cfg = { .flags = { .avoid_tearing = false } };
    lv_display_t *disp = lvgl_port_add_disp_dsi(&disp_cfg, &dsi_cfg);

    // Touch
    esp_lcd_touch_handle_t tp;
    ESP_ERROR_CHECK(bsp_i2c_init());
    esp_lcd_panel_io_handle_t tp_io_handle = NULL;
    esp_lcd_panel_io_i2c_config_t tp_io_config = ESP_LCD_TOUCH_IO_I2C_GT911_CONFIG();
    tp_io_config.scl_speed_hz = 400000;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i2c(i2c_handle, &tp_io_config, &tp_io_handle));

    const esp_lcd_touch_config_t tp_cfg = {
        .x_max = BSP_LCD_H_RES,
        .y_max = BSP_LCD_V_RES,
        .rst_gpio_num = GPIO_NUM_NC,
        .int_gpio_num = GPIO_NUM_NC,
        .levels = { .reset = 0, .interrupt = 0 },
        .flags = { .swap_xy = 0, .mirror_x = 0, .mirror_y = 0 },
    };
    ESP_ERROR_CHECK(esp_lcd_touch_new_i2c_gt911(tp_io_handle, &tp_cfg, &tp));

    const lvgl_port_touch_cfg_t touch_cfg = { .disp = disp, .handle = tp };
    lvgl_port_add_touch(&touch_cfg);

    return disp;
}

bool bsp_display_lock(uint32_t timeout_ms) { return lvgl_port_lock(timeout_ms); }
void bsp_display_unlock(void) { lvgl_port_unlock(); }

esp_err_t bsp_extra_codec_init() { return ESP_OK; }
