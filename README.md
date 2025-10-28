# TPanelS3ArduinoSupport

Arduino support library for TPanel S3 with integrated display drivers.

## Features
- Integrated Arduino_GFX (v1.4.6) - customized for TPanel S3
- TPanel S3 support
- XL95x5 Driver
- Automatic installation of LVGL and TouchLib

## Installation

### PlatformIO
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

### Arduino IDE
*Not recommended - use PlatformIO instead*

## Usage
```cpp
#include <tpanels3.h>

TPanelS3 panel;

void setup() {
    panel.initTPanelS3();
}
```

## Vendored Libraries

This library includes modified versions of:
- **Arduino_GFX** (v1.4.6) - [Original](https://github.com/moononournation/Arduino_GFX)
- **tpanels3** - Custom TPanel S3 support
- **XL95x5_Driver** - I/O Expander driver

## Dependencies

Automatically installed:
- LVGL 9.2.2
- TouchLib

## License

MIT License - See LICENSE file

## Credits

Original libraries by their respective authors.
