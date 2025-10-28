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
[env:esp32s3]
platform = espressif32
board = esp32s3_flash_16MB
framework = arduino
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
