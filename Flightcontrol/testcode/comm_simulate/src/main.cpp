/*
  Minimal FlySky iBus Receiver Emulator
  Target: ESP32-S3

  Behavior:
    - Sends valid 14-channel iBus servo frames forever
    - All channels fixed at 1500 µs equivalent
    - Frame rate ≈ 7 ms (like a real FlySky receiver)
    - No telemetry, no I2C, no extra traffic
*/

#include <Arduino.h>

// ---------- iBus constants ----------
static const uint8_t  IBUS_SERVO_FRAME_LEN = 32;  // bytes per frame
static const uint8_t  IBUS_SERVO_HDR1      = 0x20;
static const uint8_t  IBUS_SERVO_HDR2      = 0x40;
static const uint8_t  IBUS_NUM_CHANNELS    = 14;

static const uint16_t IBUS_CH_MID_US       = 1500; // neutral value

// ---------- UART for iBus ----------
HardwareSerial IBUSSerial(1);  // use UART1

// Adjust these to match your wiring
static const int IBUS_RX_PIN = 22;   // Emulator RX  <- FC TX (optional)
static const int IBUS_TX_PIN = 23;   // Emulator TX  -> FC RX

// ---------- Channel buffer ----------
static uint16_t g_channels[IBUS_NUM_CHANNELS];

// ---------- Timing ----------
static uint32_t g_lastServoMs   = 0;
static const uint16_t SERVO_PERIOD_MS = 7;  // ~7 ms like real receiver

// ---------- Helpers ----------
static void setAllChannels(uint16_t value) {
  for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; ++i) {
    g_channels[i] = value;
  }
}

// Build and send one complete iBus servo frame
static void sendServoFrame() {
  uint8_t frame[IBUS_SERVO_FRAME_LEN];

  frame[0] = IBUS_SERVO_HDR1;  // 0x20 = length (32 bytes)
  frame[1] = IBUS_SERVO_HDR2;  // 0x40 = servo frame command

  // 14 channels, little-endian
  for (uint8_t i = 0; i < IBUS_NUM_CHANNELS; ++i) {
    uint16_t v = g_channels[i];
    frame[2 + 2 * i]     = (uint8_t)(v & 0xFF);        // low byte
    frame[2 + 2 * i + 1] = (uint8_t)((v >> 8) & 0xFF); // high byte
  }

  // Checksum = 0xFFFF - sum(bytes[0..29])
  uint16_t sum = 0;
  for (uint8_t i = 0; i < 30; ++i) {
    sum += frame[i];
  }
  uint16_t csum = 0xFFFFu - sum;

  frame[30] = (uint8_t)(csum & 0xFF);       // CRC low
  frame[31] = (uint8_t)((csum >> 8) & 0xFF); // CRC high

  IBUSSerial.write(frame, IBUS_SERVO_FRAME_LEN);
}

// ---------- Arduino setup/loop ----------
void setup() {
  // Optional debug over USB
  Serial.begin(115200);
  delay(1000);
  Serial.println("iBus RX emulator: 14 channels @ 1500, 7 ms period");

  // Start UART1 for iBus
  IBUSSerial.begin(115200, SERIAL_8N1, IBUS_RX_PIN, IBUS_TX_PIN);

  // Initialize channels to neutral
  setAllChannels(IBUS_CH_MID_US);

  g_lastServoMs = millis();
}

void loop() {
  uint32_t now = millis();

  // Periodic servo frames only (no telemetry spam)
  if ((uint16_t)(now - g_lastServoMs) >= SERVO_PERIOD_MS) {
    g_lastServoMs = now;
    sendServoFrame();
  }

  // No delay(); keep timing tight and regular
}