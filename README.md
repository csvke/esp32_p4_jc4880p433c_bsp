# ESP32-P4 JC4880P433C BSP Component

ESP-IDF Board Support Package (BSP) for the JC4880P433C development board with ESP32-P4 microcontroller.

## Hardware Features

- **MCU**: ESP32-P4 RISC-V dual-core processor
- **Display**: 4.3" 480x800 ST7701 MIPI-DSI LCD
- **Touch**: GT911 capacitive touch controller
- **Backlight**: PWM controlled via GPIO23
- **Interface**: MIPI-DSI display interface
- **Power**: Integrated power management

## Supported Features

- LCD display initialization and control
- Touch screen input handling
- PWM backlight brightness control
- GPIO configuration for board peripherals
- LVGL graphics library integration

## Installation

### Using ESP-IDF Component Manager

Add this component to your project's `idf_component.yml`:

```yaml
dependencies:
  csvke/esp32_p4_jc4880p433c_bsp: "^1.0.0"
```

### Manual Installation

Clone this repository into your project's `components` directory:

```bash
cd your_project/components
git clone https://github.com/csvke/esp32_p4_jc4880p433c_bsp.git
```

## Usage

```c
#include "jc4880p4_bsp.h"

void app_main(void)
{
    // Initialize the BSP
    bsp_init();
    
    // Initialize display
    bsp_display_init();
    
    // Set backlight brightness (0-100%)
    bsp_display_brightness_set(80);
    
    // Your application code here
}
```

## API Reference

### Display Functions

- `esp_err_t bsp_display_init(void)` - Initialize the display
- `esp_err_t bsp_display_brightness_set(int brightness_percent)` - Set backlight brightness (0-100%)
- `lv_display_t* bsp_display_get_lvgl_disp(void)` - Get LVGL display object

### Touch Functions

- `esp_err_t bsp_touch_init(void)` - Initialize touch controller
- `lv_indev_t* bsp_touch_get_lvgl_indev(void)` - Get LVGL input device

### General Functions

- `esp_err_t bsp_init(void)` - Initialize all BSP components

## Configuration

The BSP can be configured via ESP-IDF menuconfig under `Component config → Board Support Package`.

## Hardware Schematic

Refer to the manufacturer documentation for detailed hardware schematics and pin assignments.

## Dependencies

- ESP-IDF v5.0 or later
- esp_lcd_st7701 component
- esp_lcd_touch_gt911 component  
- esp_lvgl_port component

## License

This project is licensed under the Apache License 2.0 - see the LICENSE file for details.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.