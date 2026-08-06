#include <Arduino.h>

namespace {

constexpr uint32_t kUsbBaud = 115200;
constexpr uint32_t kODriveBaud = 115200;
constexpr uint32_t kStartupDelayMs = 1000;
constexpr uint32_t kQueryPeriodMs = 1000;
constexpr uint32_t kResponseWindowMs = 250;

uint32_t last_query_ms = 0;
uint32_t query_sent_ms = 0;
uint32_t response_byte_count = 0;
bool awaiting_response = false;
uint8_t query_index = 0;

uint8_t xorChecksum(const char* text) {
  uint8_t checksum = 0;
  while (*text != '\0') {
    checksum ^= static_cast<uint8_t>(*text);
    ++text;
  }
  return checksum;
}

void sendReadOnlyQuery() {
  const char* label = nullptr;
  const char* body = nullptr;
  bool checksummed = false;
  switch (query_index) {
    case 0:
      label = "plain bus voltage";
      body = "r vbus_voltage";
      break;
    case 1:
      label = "checksummed bus voltage";
      body = "r vbus_voltage ";
      checksummed = true;
      break;
    case 2:
      label = "checksummed active errors";
      body = "r axis0.active_errors ";
      checksummed = true;
      break;
    default:
      label = "checksummed position/velocity feedback";
      body = "f 0 ";
      checksummed = true;
      break;
  }

  Serial1.print(body);
  if (checksummed) {
    Serial1.print('*');
    Serial1.print(xorChecksum(body));
  }
  Serial1.print('\n');

  query_sent_ms = millis();
  last_query_ms = query_sent_ms;
  response_byte_count = 0;
  awaiting_response = true;
  Serial.print("TEST: ");
  Serial.print(label);
  Serial.print("\r\nRX <- ODrive: ");
  query_index = static_cast<uint8_t>((query_index + 1U) % 4U);
}

}  // namespace

void setup() {
  Serial.begin(kUsbBaud);
  Serial1.begin(kODriveBaud);  // Teensy 4.1 pin 0=RX1, pin 1=TX1.
  delay(kStartupDelayMs);

  Serial.println();
  Serial.println("ODrive S1 read-only UART probe");
  Serial.println("Serial1: 115200 baud, Teensy pin 0 RX1 / pin 1 TX1");
  Serial.println("Only read-only voltage, error, and feedback queries are sent.");
  Serial.println();
  sendReadOnlyQuery();
}

void loop() {
  while (Serial1.available() > 0) {
    const int incoming = Serial1.read();
    if (incoming >= 0) {
      Serial.write(static_cast<uint8_t>(incoming));
      ++response_byte_count;
    }
  }

  const uint32_t now_ms = millis();
  if (awaiting_response && now_ms - query_sent_ms >= kResponseWindowMs) {
    if (response_byte_count == 0U) {
      Serial.println("<no bytes received>");
    } else {
      Serial.println();
    }
    awaiting_response = false;
  }

  if (!awaiting_response && now_ms - last_query_ms >= kQueryPeriodMs) {
    sendReadOnlyQuery();
  }
}
