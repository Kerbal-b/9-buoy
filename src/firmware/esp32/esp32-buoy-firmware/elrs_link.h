#pragma once

#include <Arduino.h>
#include <HardwareSerial.h>
#include <string.h>

// SkyGuy Nano receiver UART is configured for ExpressLRS MAVLink mode.

class ElrsLink {
 public:
  static constexpr uint8_t kSystemId = 1;
  static constexpr uint8_t kComponentId = 1;
  static constexpr uint32_t kRcTimeoutMs = 500;

  explicit ElrsLink(HardwareSerial& serial) : serial_(serial) {}

  void begin() {
    serial_.setRxBufferSize(512);
    serial_.begin(460800, SERIAL_8N1, 34, 25);
  }

  void poll() {
    // Bound work per loop so a noisy receiver cannot starve motor safety.
    for (unsigned count = 0; count < 256 && serial_.available() > 0; ++count) {
      accept(static_cast<uint8_t>(serial_.read()));
    }
  }

  bool rcFresh(uint32_t now) const {
    return lastRcMs_ != 0 && now - lastRcMs_ <= kRcTimeoutMs;
  }

  bool linkFresh(uint32_t now) const {
    return lastLinkMs_ != 0 && now - lastLinkMs_ <= 2000;
  }

  uint16_t channel(uint8_t index) const { return index < 16 ? channels_[index] : 0; }
  uint32_t lastRcMs() const { return lastRcMs_; }
  uint8_t linkQuality() const { return linkQuality_; }
  int8_t rssiDbm() const { return rssiDbm_; }
  int8_t snrDb() const { return snrDb_; }
  uint32_t receivedBytes() const { return receivedBytes_; }
  uint32_t validRadioStatus() const { return validRadioStatus_; }
  uint32_t validRcFrames() const { return validRcFrames_; }
  uint32_t crcErrors() const { return crcErrors_; }

  void sendHeartbeat(bool active) {
    uint8_t payload[9] = {};
    payload[4] = 11;  // MAV_TYPE_SURFACE_BOAT
    payload[5] = 8;   // MAV_AUTOPILOT_INVALID
    payload[6] = active ? 0x80 : 0;  // MAV_MODE_FLAG_SAFETY_ARMED
    payload[7] = active ? 4 : 3;     // ACTIVE / STANDBY
    payload[8] = 3;
    sendMavlink(0, 50, payload, sizeof(payload), kComponentId);
  }

  void sendBattery(float volts, float amps, int percent) {
    if (!isfinite(volts) || volts <= 0) return;
    volts = constrain(volts, 0.0f, 65.0f);
    amps = isfinite(amps) ? constrain(amps, 0.0f, 327.0f) : 0.0f;
    percent = constrain(percent, 0, 100);
    // SYS_STATUS includes measured battery fields; other health fields are zero.
    uint8_t payload[31] = {};
    putLe16(payload + 14, static_cast<uint16_t>(roundf(volts * 1000.0f)));
    putLe16(payload + 16, static_cast<uint16_t>(roundf(amps * 100.0f)));
    payload[30] = static_cast<uint8_t>(percent);
    sendMavlink(1, 124, payload, sizeof(payload), kComponentId);
  }

  void sendGps(double latitude, double longitude, float speedMps,
               float courseDegrees, float altitudeMeters, uint8_t satellites) {
    if (!isfinite(latitude) || !isfinite(longitude) ||
        latitude < -90 || latitude > 90 || longitude < -180 || longitude > 180) return;
    int32_t latE7 = static_cast<int32_t>(round(latitude * 1e7));
    int32_t lonE7 = static_cast<int32_t>(round(longitude * 1e7));
    uint8_t payload[28] = {};
    putLe32(payload + 4, static_cast<uint32_t>(latE7));
    putLe32(payload + 8, static_cast<uint32_t>(lonE7));
    putLe32(payload + 12, static_cast<uint32_t>(static_cast<int32_t>(roundf(altitudeMeters * 1000.0f))));
    putLe16(payload + 26, static_cast<uint16_t>(constrain(roundf(courseDegrees * 100.0f), 0.0f, 35999.0f)));
    sendMavlink(33, 104, payload, sizeof(payload), kComponentId);
  }

  void sendLinkAndChannels(uint32_t now) {
    if (linkFresh(now) && now - lastLinkSendMs_ >= 1000) {
      uint8_t payload[9] = {};
      payload[4] = static_cast<uint8_t>((static_cast<uint16_t>(linkQuality_) * 255 + 50) / 100);
      payload[5] = static_cast<uint8_t>(constrain(-static_cast<int>(rssiDbm_), 0, 255));
      payload[6] = 255;
      payload[7] = static_cast<uint8_t>(snrDb_);
      payload[8] = 255;
      sendMavlink(109, 185, payload, sizeof(payload), 68);
      lastLinkSendMs_ = now;
    }
    // Eight-channel banks fit within ELRS 50 Hz downlink bandwidth.
    if (!rcFresh(now) || now - lastChannelSendMs_ < 500) return;
    uint8_t bank = nextBank_;
    if (!bankChanged_[bank] && now - lastBankSendMs_[bank] < 6000) bank ^= 1;
    if (!bankChanged_[bank] && now - lastBankSendMs_[bank] < 6000) return;
    uint8_t payload[22] = {};
    putLe32(payload, now);
    for (uint8_t i = 0; i < 8; ++i) putLe16(payload + 4 + i * 2, channels_[bank * 8 + i]);
    payload[20] = bank;
    payload[21] = 255;
    sendMavlink(35, 244, payload, sizeof(payload), kComponentId);
    lastChannelSendMs_ = now;
    lastBankSendMs_[bank] = now;
    bankChanged_[bank] = false;
    nextBank_ = bank ^ 1;
  }

 private:
  HardwareSerial& serial_;
  uint16_t channels_[16] = {};
  uint32_t lastRcMs_ = 0;
  uint32_t lastLinkMs_ = 0;
  uint8_t linkQuality_ = 0;
  int8_t rssiDbm_ = 0;
  int8_t snrDb_ = 0;
  uint8_t frame_[280] = {};
  uint16_t frameLength_ = 0;
  uint16_t frameExpected_ = 0;
  uint8_t sequence_ = 0;
  uint32_t lastChannelSendMs_ = 0;
  uint32_t lastLinkSendMs_ = 0;
  uint32_t lastBankSendMs_[2] = {};
  bool bankChanged_[2] = {true, true};
  uint8_t nextBank_ = 0;
  uint32_t receivedBytes_ = 0;
  uint32_t validRadioStatus_ = 0;
  uint32_t validRcFrames_ = 0;
  uint32_t crcErrors_ = 0;

  static void putLe16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
  }
  static void putLe32(uint8_t* out, uint32_t value) {
    for (uint8_t i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
  }
  static void putBe16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value >> 8);
    out[1] = static_cast<uint8_t>(value);
  }
  static void putBe32(uint8_t* out, uint32_t value) {
    for (uint8_t i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (24 - 8 * i));
  }

  static uint16_t mavCrc(uint16_t crc, uint8_t value) {
    uint8_t tmp = value ^ (crc & 0xff);
    tmp ^= tmp << 4;
    return (crc >> 8) ^ (uint16_t(tmp) << 8) ^ (uint16_t(tmp) << 3) ^ (tmp >> 4);
  }
  void sendMavlink(uint8_t messageId, uint8_t extra, const uint8_t* payload,
                   uint8_t length, uint8_t component) {
    uint8_t frame[263] = {0xFE, length, sequence_++, kSystemId, component, messageId};
    memcpy(frame + 6, payload, length);
    uint16_t crc = 0xffff;
    for (uint16_t i = 1; i < 6 + length; ++i) crc = mavCrc(crc, frame[i]);
    crc = mavCrc(crc, extra);
    frame[6 + length] = static_cast<uint8_t>(crc);
    frame[7 + length] = static_cast<uint8_t>(crc >> 8);
    serial_.write(frame, length + 8);
  }
  void accept(uint8_t value) {
    ++receivedBytes_;
    if (frameLength_ == 0) {
      if (value != 0xFE && value != 0xFD) return;
      frame_[frameLength_++] = value;
      frameExpected_ = 0;
      return;
    }
    if (frameLength_ >= sizeof(frame_)) { frameLength_ = 0; return; }
    frame_[frameLength_++] = value;
    bool v2 = frame_[0] == 0xFD;
    uint8_t headerLength = v2 ? 10 : 6;
    if (frameLength_ == headerLength) {
      uint8_t signatureLength = v2 && (frame_[2] & 1) ? 13 : 0;
      frameExpected_ = headerLength + frame_[1] + 2 + signatureLength;
      if (frameExpected_ > sizeof(frame_)) frameLength_ = 0;
    }
    if (!frameExpected_ || frameLength_ < frameExpected_) return;
    uint32_t messageId = v2
        ? uint32_t(frame_[7]) | (uint32_t(frame_[8]) << 8) | (uint32_t(frame_[9]) << 16)
        : frame_[5];
    uint8_t extra = messageId == 109 ? 185 : (messageId == 70 ? 124 : 0);
    uint8_t length = frame_[1];
    if (extra && (messageId == 109 || (messageId == 70 && length >= 17))) {
      uint16_t crc = 0xffff;
      for (uint16_t i = 1; i < headerLength + length; ++i) crc = mavCrc(crc, frame_[i]);
      crc = mavCrc(crc, extra);
      uint16_t at = headerLength + length;
      if (frame_[at] == uint8_t(crc) && frame_[at + 1] == uint8_t(crc >> 8)) {
        const uint8_t* p = frame_ + headerLength;
        if (messageId == 109) {
          ++validRadioStatus_;
          // MAVLink 2 omits trailing zero bytes, including fixed fields.
          const uint8_t lq = length > 4 ? p[4] : 0;
          const uint8_t rssi = length > 5 ? p[5] : 0;
          const uint8_t snr = length > 7 ? p[7] : 0;
          linkQuality_ = static_cast<uint8_t>((uint16_t(lq) * 100 + 127) / 255);
          rssiDbm_ = -static_cast<int8_t>(constrain(static_cast<int>(rssi), 0, 127));
          snrDb_ = static_cast<int8_t>(snr);
          lastLinkMs_ = millis();
        } else if (p[16] == kSystemId &&
                   (length == 17 || p[17] == 0 || p[17] == kComponentId)) {
          // 0 and UINT16_MAX mean ignored channels in RC_CHANNELS_OVERRIDE.
          bool valid = true;
          for (uint8_t i = 0; i < 5; ++i) {
            uint16_t us = uint16_t(p[i * 2]) | (uint16_t(p[i * 2 + 1]) << 8);
            if (us < 900 || us > 2100) { valid = false; break; }
          }
          if (valid) {
            ++validRcFrames_;
            for (uint8_t i = 0; i < 16; ++i) {
              uint8_t offset = i < 8 ? i * 2 : 18 + (i - 8) * 2;
              uint16_t us = offset + 1 < length
                  ? uint16_t(p[offset]) | (uint16_t(p[offset + 1]) << 8)
                  : 0;
              if (us == 65535) us = 0;
              if (channels_[i] != us) bankChanged_[i / 8] = true;
              channels_[i] = us;
            }
            lastRcMs_ = millis();
          }
        }
      } else {
        ++crcErrors_;
      }
    }
    frameLength_ = 0;
    frameExpected_ = 0;
  }
};
