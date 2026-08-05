#pragma once

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>

namespace odrive_ascii {

inline uint8_t checksum(const char* text, const size_t length) {
  uint8_t result = 0;
  for (size_t index = 0; index < length; ++index) {
    result ^= static_cast<uint8_t>(text[index]);
  }
  return result;
}

// Validates an ODrive response of the form "payload *42" and replaces the
// space before '*' with a null terminator so callers can parse payload safely.
inline bool validateAndStripChecksum(char* line) {
  if (line == nullptr) return false;
  char* star = std::strrchr(line, '*');
  if (star == nullptr || star == line) return false;
  char* end = nullptr;
  const long received = std::strtol(star + 1, &end, 10);
  if (end == star + 1 || *end != '\0' || received < 0 || received > 255) {
    return false;
  }
  const size_t checksummed_length = static_cast<size_t>(star - line);
  if (checksum(line, checksummed_length) != static_cast<uint8_t>(received)) {
    return false;
  }
  size_t payload_length = checksummed_length;
  while (payload_length > 0U && line[payload_length - 1U] == ' ') {
    --payload_length;
  }
  line[payload_length] = '\0';
  return true;
}

inline bool parseFeedback(char* payload, float& position, float& velocity) {
  if (payload == nullptr) return false;
  char* end = nullptr;
  position = std::strtof(payload, &end);
  if (end == payload) return false;
  while (*end == ' ') ++end;
  char* velocity_end = nullptr;
  velocity = std::strtof(end, &velocity_end);
  if (velocity_end == end) return false;
  while (*velocity_end == ' ') ++velocity_end;
  return *velocity_end == '\0' && std::isfinite(position) &&
         std::isfinite(velocity);
}

}  // namespace odrive_ascii
