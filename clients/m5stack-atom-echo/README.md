# M5Stack Atom Echo - Owl AI Wearable

This directory contains the firmware for running the Owl AI wearable on the M5Stack Atom Echo device.

## Hardware Overview

The M5Stack Atom Echo is a compact ESP32-PICO based device with:
- **ESP32-PICO-D4** microcontroller
- **Built-in microphone** (SPH0645LM4H-B)
- **Built-in speaker** with I2S amplifier
- **RGB LED** (WS2812B)
- **Physical button**
- **USB-C connector** for power and programming
- **Compact form factor** (24×24×10mm)

## Pin Configuration

The firmware has been configured for the M5Stack Atom Echo pin layout:

| Function | Pin | Description |
|----------|-----|-------------|
| I2S BCLK | G19 | Bit Clock for I2S communication |
| I2S LRCK | G33 | Left/Right Clock for I2S |
| I2S DataOut | G22 | Speaker output (not used for mic recording) |
| I2S DataIn | G23 | Microphone input |
| Button | G39 | Physical button input |
| RGB LED | G27 | WS2812B RGB LED |

## Features

- **Audio Capture**: 16kHz mono audio recording from built-in microphone
- **AAC Encoding**: Real-time AAC encoding for efficient data transmission
- **Bluetooth LE**: Streams audio data to iOS app via BLE
- **RGB LED Status**: Visual feedback for connection status and button presses
- **Button Input**: Physical interaction with debounced button handling

## LED Status Indicators

- **Blue**: Device startup/initialization
- **Red**: Disconnected, waiting for connection
- **Green**: Connected to iOS app
- **White Flash**: Button pressed
- **Red Flashing**: Audio initialization error

## Setup Instructions

### 1. Install PlatformIO

First, install [PlatformIO](https://platformio.org/) if you haven't already:

```bash
# Install PlatformIO Core
pip install platformio

# Or use PlatformIO IDE extension in VS Code
```

### 2. Copy Required Libraries

The audio codec library needs to be manually copied:

```bash
# Navigate to the M5Stack Atom Echo firmware directory
cd clients/m5stack-atom-echo/firmware

# Copy the ESP32 audio codec library
cp ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/lib/esp32/libesp_audio_codec.a \
   lib/espressif_esp_audio_codec_1.0.1/lib/esp32/

# Copy other library files
cp ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/CMakeLists.txt \
   lib/espressif_esp_audio_codec_1.0.1/
cp ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/LICENSE \
   lib/espressif_esp_audio_codec_1.0.1/
cp ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/idf_component.yml \
   lib/espressif_esp_audio_codec_1.0.1/
```

### 3. Build and Upload

```bash
# Build the firmware
pio run

# Upload to M5Stack Atom Echo (device must be connected via USB-C)
pio run --target upload

# Monitor serial output
pio device monitor
```

### 4. Connect to iOS App

1. The device will advertise as "m5stack_atom_echo" over Bluetooth LE
2. Open the Owl iOS app
3. The device should appear in the BLE device list
4. Connect and start streaming audio

## Troubleshooting

### Build Issues

- **Library not found**: Ensure the ESP32 audio codec library has been copied correctly
- **Board not found**: Check that PlatformIO has the M5Stack Atom board definitions installed
- **Compilation errors**: Verify all dependencies are installed (`FastLED`, `ESP32 I2S library`)

### Runtime Issues

- **LED stuck on blue**: Audio initialization failed, check serial monitor
- **No BLE advertising**: Device may be in a crash loop, check serial monitor
- **Audio quality issues**: Verify microphone connections and I2S configuration

### Serial Monitor Commands

The device outputs debug information to serial at 115200 baud:

```
M5Stack Atom Echo - Owl AI Wearable
RGB LED initialized
Button initialized on pin 39
MTU size: 185 bytes
AAC encoder initialized
Microphone initialized
Setup completed - Ready for connection
```

## Hardware Modifications

The M5Stack Atom Echo works out of the box, but for optimal performance:

1. **Power Supply**: Use a stable 5V power source for continuous operation
2. **Case/Mounting**: Consider a protective case for wearable use
3. **Antenna**: The built-in antenna should be sufficient for most applications

## Differences from XIAO ESP32S3 Sense

| Feature | XIAO ESP32S3 Sense | M5Stack Atom Echo |
|---------|-------------------|-------------------|
| MCU | ESP32-S3 | ESP32-PICO-D4 |
| Microphone | PDM (pins 42,41) | I2S (pins 19,33,23) |
| Button | D0 | G39 |
| LED | Built-in | WS2812B RGB (G27) |
| Speaker | None | Built-in I2S speaker |
| Form Factor | Development board | Compact cube |

## Advanced Configuration

### Audio Settings

The audio configuration can be modified in `src/main.cpp`:

```cpp
constexpr const int SampleRate = 16000;  // Sample rate in Hz
s_encoder_config.bitrate = 90000;        // AAC bitrate in bps
```

### BLE Settings

BLE characteristics and advertising can be customized:

```cpp
constexpr const char *ServiceID = "03d5d5c4-a86c-11ee-9d89-8f2089a49e7e";
BLEDevice::init("m5stack_atom_echo");  // Device name
```

### LED Brightness

RGB LED brightness can be adjusted:

```cpp
FastLED.setBrightness(50);  // 0-255, default is 50
```

## Contributing

When contributing improvements:

1. Test thoroughly on actual M5Stack Atom Echo hardware
2. Maintain compatibility with the existing Owl server and iOS app
3. Document any configuration changes
4. Follow the existing code style and structure

## License

This firmware is part of the Owl project and follows the same MIT license as the main project.