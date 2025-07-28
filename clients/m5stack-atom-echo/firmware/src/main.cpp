/*
 * Header format:
 *
 *          Bit
 *  Byte     7  4 3  0
 *          +----+----+
 *    0     |xxxx|ffff|
 *          +----+----+
 * 
 *          +----+----+
 *    1     |nnnn|ssss|
 *          +----+----+
 * 
 *  xxxx    Reserved
 *  ffff    Inter-frame sequence number (complete frame)
 *  nnnn    Number of BLE packets in this frame
 *  ssss    Sequence number for this frame (intra-frame), [0,n)
 */

#include <Arduino.h>
#include <I2S.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <FastLED.h>
#include "esp_aac_enc.h"
#include "esp_audio_enc_def.h"
#include "esp_audio_def.h"

// M5Stack Atom Echo Pin Configuration
#define I2S_BCLK_PIN    19  // G19 - BCLK SPK-I2S
#define I2S_LRCK_PIN    33  // G33 - LRCK
#define I2S_DATA_OUT    22  // G22 - DataOut SPK-I2S (not used for mic recording)
#define I2S_DATA_IN     23  // G23 - DataIn/MIC
#define BUTTON_PIN      39  // G39 - Button
#define LED_PIN         27  // G27 - RGB LED
#define NUM_LEDS        1   // M5Stack Atom Echo has 1 RGB LED

// BLE Configuration
constexpr const char *ServiceID = "03d5d5c4-a86c-11ee-9d89-8f2089a49e7e";
constexpr const char *TxID = "b189a505-a86c-11ee-a5fb-8f2089a49e7e";
constexpr const char *RxID = "ff000353-a872-11ee-b751-8f2089a49e7e";

constexpr const size_t HeaderSize = 2;    // sequence numbers
constexpr const size_t MaxSendSize = 182; // match iOS MTU size (185 - 3) for best send rate

// Global variables
static BLECharacteristic *s_tx = nullptr;
static bool s_is_connected = false;
static uint8_t s_output_buffer[MaxSendSize];
static uint8_t s_interframe_seqno = 0;
static uint8_t s_intraframe_seqno = 0;

constexpr const int SampleRate = 16000;
static int8_t s_audio_buffer[2048];

static esp_aac_enc_config_t s_encoder_config;
static void *s_encoder_handle = nullptr;
static int s_frame_in_bytes = 0;
static int s_frame_out_bytes = 0;
static uint8_t *s_recording_buffer = nullptr;
static uint8_t *s_input_frame = nullptr;
static uint8_t *s_compressed_frame = nullptr;

// LED configuration
CRGB leds[NUM_LEDS];
bool button_pressed = false;
unsigned long last_button_press = 0;
const unsigned long debounce_delay = 50;

class server_handler: public BLEServerCallbacks
{
  void onConnect(BLEServer *server)
  {
    s_is_connected = true;
    Serial.println("Connected");
    // Set LED to green when connected
    leds[0] = CRGB::Green;
    FastLED.show();
  }

  void onDisconnect(BLEServer *server)
  {
    s_is_connected = false;
    Serial.println("Disconnected");
    // Set LED to red when disconnected
    leds[0] = CRGB::Red;
    FastLED.show();
    BLEDevice::startAdvertising();
  }
};

class message_handler: public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic* pCharacteristic, esp_ble_gatts_cb_param_t* param)
  {
    // Currently unused
  }
};

static void setup_rgb_led()
{
  FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.setBrightness(50); // Set moderate brightness
  
  // Initialize with blue color (startup)
  leds[0] = CRGB::Blue;
  FastLED.show();
  
  Serial.println("RGB LED initialized");
}

static void setup_button()
{
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  Serial.println("Button initialized on pin 39");
}

static void setup_microphone()
{
  // Configure I2S for M5Stack Atom Echo
  // For INMP441 microphone: SCK=BCLK, WS=LRCK, SD=DATA_IN
  I2S.setAllPins(I2S_DATA_IN, I2S_BCLK_PIN, I2S_LRCK_PIN, -1, -1);
  
  if (!I2S.begin(PDM_MONO_MODE, SampleRate, 16))
  {
      while (true)
      {
        Serial.println("Failed to initialize I2S for audio recording!");
        // Flash LED red to indicate error
        leds[0] = CRGB::Red;
        FastLED.show();
        delay(500);
        leds[0] = CRGB::Black;
        FastLED.show();
        delay(500);
      }
  }
  
  Serial.println("Microphone initialized");
}

static void setup_aac_encoder()
{
  // Set encoder configuration
  s_encoder_config = ESP_AAC_ENC_CONFIG_DEFAULT();
  s_encoder_config.sample_rate = 16000;
  s_encoder_config.channel = 1;
  s_encoder_config.bitrate = 90000;
  s_encoder_config.adts_used = 1;

  // Create encoder
  if (0 != esp_aac_enc_open(&s_encoder_config, sizeof(esp_aac_enc_config_t), &s_encoder_handle))
  {
    Serial.println("Error: Unable to create encoder");
    return;
  }
  
  // Get size of input and output frames
  esp_aac_enc_get_frame_size(s_encoder_handle, &s_frame_in_bytes, &s_frame_out_bytes);
  Serial.printf("Frame in: %d bytes\n", s_frame_in_bytes);
  Serial.printf("Frame out: %d bytes\n", s_frame_out_bytes);
      
  // Allocate audio buffers
  s_recording_buffer = (uint8_t *) ps_calloc(s_frame_in_bytes, sizeof(uint8_t));
  s_input_frame = (uint8_t *) ps_calloc(s_frame_in_bytes, sizeof(uint8_t));
  s_compressed_frame = (uint8_t *) ps_calloc(s_frame_out_bytes, sizeof(uint8_t));
  
  Serial.println("AAC encoder initialized");
}

static void handle_button()
{
  bool current_button_state = !digitalRead(BUTTON_PIN); // Inverted because pullup
  unsigned long current_time = millis();
  
  if (current_button_state && !button_pressed && (current_time - last_button_press > debounce_delay))
  {
    button_pressed = true;
    last_button_press = current_time;
    
    // Button pressed - flash LED white
    leds[0] = CRGB::White;
    FastLED.show();
    delay(100);
    
    // Restore connection status color
    if (s_is_connected) {
      leds[0] = CRGB::Green;
    } else {
      leds[0] = CRGB::Red;
    }
    FastLED.show();
    
    Serial.println("Button pressed");
  }
  else if (!current_button_state && button_pressed)
  {
    button_pressed = false;
    Serial.println("Button released");
  }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("M5Stack Atom Echo - Owl AI Wearable");
  
  // Initialize hardware
  setup_rgb_led();
  setup_button();
  
  // Initialize BLE
  BLEDevice::init("m5stack_atom_echo");
  BLEServer *server = BLEDevice::createServer();
  BLEService *service = server->createService(ServiceID);
  s_tx = service->createCharacteristic(
    TxID,
    BLECharacteristic::PROPERTY_NOTIFY
  );
  BLECharacteristic *rx = service->createCharacteristic(
    RxID,
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  rx->setCallbacks(new message_handler());
  server->setCallbacks(new server_handler());
  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(ServiceID);
  advertising->setScanResponse(true);
  advertising->setMinPreferred(0x06);
  advertising->setMinPreferred(0x12);
  advertising->setMinInterval(0x20);
  advertising->setMaxInterval(0x40);
  BLEDevice::startAdvertising();

  Serial.printf("MTU size: %d bytes\n", BLEDevice::getMTU());

  // Initialize audio components
  setup_aac_encoder();
  setup_microphone();

  // Set LED to red (waiting for connection)
  leds[0] = CRGB::Red;
  FastLED.show();

  Serial.println("Setup completed - Ready for connection");
}

void loop()
{
  // Handle button input
  handle_button();
  
  if (!s_is_connected) {
    delay(50); // Wait for a connection
    return;
  }
  
  unsigned long t0 = millis();

  size_t bytes_recorded = 0;
  esp_i2s::i2s_read(esp_i2s::I2S_NUM_0, s_recording_buffer, s_frame_in_bytes, &bytes_recorded, portMAX_DELAY);
  if (0 == bytes_recorded)
  {
    Serial.println("Recording failed!");
    return;
  }

  unsigned long t1 = millis();
  //Serial.printf("Recording took: %lu\n", t1 - t0);

  // Encode process
  const uint8_t *recording = s_recording_buffer;
  const uint8_t *recording_end = s_recording_buffer + bytes_recorded;
  esp_audio_enc_in_frame_t in_frame = { 0 };
  esp_audio_enc_out_frame_t out_frame = { 0 };
  in_frame.buffer = s_input_frame;
  in_frame.len = s_frame_in_bytes;
  out_frame.buffer = s_compressed_frame;
  out_frame.len = s_frame_out_bytes;

  while (recording < recording_end)
  {
    unsigned long t0 = millis();

    // Don't read past end of input buffer and pad frame with zeros if need be
    int bytes_remaining = recording_end - recording;
    int chunk_bytes = min(s_frame_in_bytes, bytes_remaining);
    memcpy(s_input_frame, recording, chunk_bytes);
    if (chunk_bytes < s_frame_in_bytes)
    {
      memset(s_input_frame + chunk_bytes, 0, s_frame_in_bytes - chunk_bytes);
    }
    recording += chunk_bytes;

    if (ESP_AUDIO_ERR_OK != esp_aac_enc_process(s_encoder_handle, &in_frame, &out_frame))
    {
      Serial.println("Audio encoder process failed.");
      return;
    }

    //Serial.printf("%d -> %d\n", s_frame_in_bytes, out_frame.encoded_bytes);

    unsigned long t1 = millis();

    {
      // Compute number of packets to send out this frame
      size_t max_chunk_size = MaxSendSize - HeaderSize;
      size_t num_packets = out_frame.encoded_bytes / max_chunk_size + ((out_frame.encoded_bytes % max_chunk_size) ? 1 : 0);

      // Header: inter-frame sequence number
      s_output_buffer[0] = s_interframe_seqno & 0xf;
      s_intraframe_seqno = 0;

      // Stream out the packets
      size_t bytes_remaining = out_frame.encoded_bytes;
      uint8_t *buffer = (uint8_t *) out_frame.buffer;
      while (bytes_remaining > 0)
      {
        // Header: intra-frame sequence number
        s_output_buffer[1] = (num_packets << 4) | (s_intraframe_seqno & 0xf);

        // Copy into output buffer
        size_t chunk_size = min(max_chunk_size, bytes_remaining);
        size_t packet_size = chunk_size + HeaderSize;
        memcpy(&s_output_buffer[HeaderSize], buffer, chunk_size);
        
        // Send
        s_tx->setValue(s_output_buffer, packet_size);
        s_tx->notify();
        delay(4);

        // Next bytes
        bytes_remaining -= chunk_size;
        buffer += chunk_size;
        s_intraframe_seqno += 1;
      }

      // Frame complete
      s_interframe_seqno += 1;
    }

    unsigned long t2 = millis();
    //Serial.printf("Encoding took: %lu\n", t1 - t0);
    //Serial.printf("Sending took: %lu\n", t2 - t1);
  }
}