# ESP Audio Codec Library for M5Stack Atom Echo

This directory should contain the ESP Audio Codec library files needed for audio encoding on the M5Stack Atom Echo.

## Required Files

You need to copy the ESP32 version of the compiled library from the original XIAO ESP32S3 project:

```bash
# Copy the ESP32 library (M5Stack Atom Echo uses ESP32-PICO, which is regular ESP32)
cp ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/lib/esp32/libesp_audio_codec.a lib/esp32/

# Also copy any other needed library files:
cp -r ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/CMakeLists.txt .
cp -r ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/LICENSE .
cp -r ../../../xiao-esp32s3-sense/firmware/lib/espressif_esp_audio_codec_1.0.1/idf_component.yml .
```

## Directory Structure

The final structure should look like:
```
lib/espressif_esp_audio_codec_1.0.1/
├── include/
│   ├── esp_aac_enc.h
│   ├── esp_audio_enc_def.h
│   └── esp_audio_def.h
├── lib/
│   └── esp32/
│       └── libesp_audio_codec.a
├── CMakeLists.txt
├── LICENSE
├── README.md
└── idf_component.yml
```

The header files are already included, but you need to manually copy the compiled library and configuration files.