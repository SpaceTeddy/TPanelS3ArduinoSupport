# TPanelS3ArduinoSupport

Arduino support library for TPanel S3 with integrated display drivers.

## Features
- Integrated Arduino_GFX (v1.4.6) - customized for TPanel S3
- TPanel S3 support
- XL95x5 Driver
- Automatic installation of LVGL and TouchLib

## Installation

### PlatformIO
> **Important:** the `platform = espressif32 @6.12.0` pin below reflects the
> only combination actually tested on real hardware so far. `Arduino_ESP32RGBPanel`
> now has a code path for Arduino-ESP32 core 3.x (IDF 5.x, via
> `esp_lcd_rgb_panel_get_frame_buffer()` instead of the old private-struct hack),
> but that path has **not been verified on a real TPanel S3 board**. Before
> bumping the `espressif32` platform version, flash and visually confirm the
> display still comes up correctly.

```ini
[env]
platform = espressif32 @6.12.0
board = esp32s3_flash_16MB
framework = arduino
board_build.filesystem = littlefs
monitor_speed = 115200
monitor_filters = esp32_exception_decoder, time
upload_speed = 921600
board_upload.flash_size = 16MB
board_build.memory_type = qio_qspi

build_flags = 
	-D CORE_DEBUG_LEVEL=1
	-D BOARD_HAS_PSRAM
	-D ARDUINO_USB_MODE=1
	-D ARDUINO_USB_CDC_ON_BOOT=1
	-D ARDUINO_RUNNING_CORE=1
	-D ARDUINO_EVENT_RUNNING_CORE=1
	-DELEGANTOTA_USE_ASYNC_WEBSERVER=1
	-DLV_CONF_PATH=${platformio.include_dir}/lv_conf.h
	
[platformio]
boards_dir = ./boards
lib_deps = 
    https://github.com/SpaceTeddy/TPanelS3ArduinoSupport.git
```

### Testing Arduino-ESP32 core 3.x

The official PlatformIO `espressif32` registry package has historically lagged
behind on core 3.x support. The common workaround is the community
[pioarduino](https://github.com/pioarduino/platform-espressif32) fork, which
tracks current Arduino-ESP32 releases. To test the core-3.x code path added
above, swap the `platform` line for one of:

```ini
; latest stable release
platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip

; or pin an exact release, e.g.
platform = https://github.com/pioarduino/platform-espressif32/releases/download/55.03.31/platform-espressif32.zip
```

Everything else in the `[env]` block can stay the same. Double check
`board_build.memory_type` / PSRAM-related board options still apply after the
switch - board config keys have occasionally been renamed between core
2.x and 3.x releases.

### Arduino IDE
*Not recommended - use PlatformIO instead*

## Usage
```cpp
#include <tpanels3.h>

TPanelS3 tpanels3;

void setup() {
    // --- initialize display & touch hardware ---
    tpanels3.initTPanelS3();
    tpanels3.setRotation(0);     // optional: 0=portrait, 1=landscape, etc.

    // Set initial backlight brightness (raw 8-bit PWM duty cycle, 0-255; NOT a percentage)
    tpanels3.set_backlight_brightness(115);  // ~45% of max brightness
}
```

### Rendering and build options

By default LVGL renders **directly into the RGB panel's framebuffer**
(`LV_DISPLAY_RENDER_MODE_DIRECT`). The flush callback only writes the dirty
rows back from the data cache, there is no intermediate LVGL buffer and no
copy. Each rendered pixel crosses the (Quad-)PSRAM bus once instead of three
times, and about 1.4 MB of PSRAM stay free.

Direct mode needs rotation 0. `setRotation(1..3)` switches to partial
rendering with Arduino_GFX's rotating copy at runtime; `setRotation(0)`
switches back.

Override via `build_flags` if needed:

| Macro | Default | Meaning |
|---|---|---|
| `TPANELS3_LVGL_DIRECT_MODE` | `1` | `0` = always partial mode (render to a PSRAM stripe buffer, copy to the framebuffer) |
| `TPANELS3_LVGL_PARTIAL_LINES` | `120` | Height of the stripe buffer in partial mode |

## Vendored Libraries

This library includes modified versions of:
- **Arduino_GFX** (v1.4.6) - [Original](https://github.com/moononournation/Arduino_GFX)
- **tpanels3** - Custom TPanel S3 support
- **XL95x5_Driver** - I/O Expander driver

## Dependencies

Automatically installed:
- LVGL 9.5.0
- TouchLib

## License

MIT License - See LICENSE file

## Credits

Original libraries by their respective authors.
