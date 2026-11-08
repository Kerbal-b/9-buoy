#pragma once

#include <Arduino.h>

// Increment major for incompatible wiring or control protocols, minor for new
// compatible features, and patch for fixes. The date identifies this build.
constexpr uint16_t kFirmwareVersionMajor = 1;
constexpr uint16_t kFirmwareVersionMinor = 16;
constexpr uint16_t kFirmwareVersionPatch = 27;
constexpr uint32_t kFirmwareBuildDate = 20261108;

inline String firmwareVersionString() {
  return String(kFirmwareVersionMajor) + "." + String(kFirmwareVersionMinor) +
         "." + String(kFirmwareVersionPatch) + "+" + String(kFirmwareBuildDate);
}

inline String firmwareVersionStatusLine() {
  return String("TEL STATUS FIRMWARE ") + String(kFirmwareVersionMajor) + " " +
         String(kFirmwareVersionMinor) + " " + String(kFirmwareVersionPatch) +
         " " + String(kFirmwareBuildDate);
}
