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
  uint32_t lastLinkMs() const { return lastLinkMs_; }
  uint8_t linkQuality() const { return linkQuality_; }
  int8_t rssiDbm() const { return rssiDbm_; }
  int8_t snrDb() const { return snrDb_; }
  uint32_t receivedBytes() const { return receivedBytes_; }
  uint32_t validRadioStatus() const { return validRadioStatus_; }
  uint32_t validRcFrames() const { return validRcFrames_; }
  uint32_t rcOverrideFrames() const { return rcOverrideFrames_; }
  uint32_t rcOverrideCrcOk() const { return rcOverrideCrcOk_; }
  uint32_t rcOverrideCrcBad() const { return rcOverrideCrcBad_; }
  uint32_t rcOverrideShort() const { return rcOverrideShort_; }
  uint32_t rcOverrideTargetReject() const { return rcOverrideTargetReject_; }
  uint32_t rcOverrideChannelReject() const { return rcOverrideChannelReject_; }
  uint8_t lastRcRejectCode() const { return lastRcRejectCode_; }
  uint8_t lastRcTargetSystem() const { return lastRcTargetSystem_; }
  uint8_t lastRcTargetComponent() const { return lastRcTargetComponent_; }
  uint8_t lastRcInvalidChannel() const { return lastRcInvalidChannel_; }
  uint16_t lastRcInvalidValue() const { return lastRcInvalidValue_; }
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

  void sendBuoyState(int rearPwm, int leftPwm, int rightPwm, uint8_t state) {
    // NAMED_VALUE_INT is one compact MAVLink packet per second. Three signed
    // bytes represent applied motor PWM scaled to +/-127; byte 3 carries
    // mode, source, arm, science-session recording, and recovery flags. Audio
    // recording has its own BUOY_AUDIO NAMED_VALUE_INT. See ELRS-MAVLINK.md.
    uint8_t payload[18] = {};
    putLe32(payload, millis());
    const int pwm[3] = {rearPwm, leftPwm, rightPwm};
    for (uint8_t i = 0; i < 3; ++i) {
      const int magnitude = (min(abs(pwm[i]), 255) * 127 + 127) / 255;
      payload[4 + i] = static_cast<uint8_t>(pwm[i] < 0 ? -magnitude : magnitude);
    }
    payload[7] = state;
    memcpy(payload + 8, "BUOY_STATE", 10);
    sendMavlink(252, 44, payload, sizeof(payload), kComponentId);
  }

  void sendBuoyAudioState(bool recording) {
    uint8_t payload[18] = {};
    putLe32(payload, millis());
    putLe32(payload + 4, recording ? 1 : 0);
    memcpy(payload + 8, "BUOY_AUDIO", 10);
    sendMavlink(252, 44, payload, sizeof(payload), kComponentId);
  }

  void sendHomeKind(uint8_t homeKind) {
    uint8_t payload[18] = {};
    putLe32(payload, millis());
    putLe32(payload + 4, homeKind);
    memcpy(payload + 8, "HOME_KIND", 9);
    sendMavlink(252, 44, payload, sizeof(payload), kComponentId);
  }

  void sendScienceValue(const char* name, float value) {
    if (!name || !isfinite(value)) return;
    uint8_t payload[18] = {};
    putLe32(payload, millis());
    memcpy(payload + 4, &value, sizeof(value));
    strncpy(reinterpret_cast<char*>(payload + 8), name, 10);
    sendMavlink(251, 170, payload, sizeof(payload), kComponentId);
  }

  void sendAttitude(float rollRadians, float pitchRadians, float yawRadians) {
    if (!isfinite(rollRadians) || !isfinite(pitchRadians) || !isfinite(yawRadians)) return;
    uint8_t payload[28] = {};
    putLe32(payload, millis());
    memcpy(payload + 4, &rollRadians, sizeof(float));
    memcpy(payload + 8, &pitchRadians, sizeof(float));
    memcpy(payload + 12, &yawRadians, sizeof(float));
    // Angular rates are unavailable in the calibrated body frame; leave zero.
    sendMavlink(30, 39, payload, sizeof(payload), kComponentId);
  }

  void sendBattery(float volts, float amps, int percent) {
    if (!isfinite(volts) || volts <= 0) return;
    volts = constrain(volts, 0.0f, 65.0f);
    amps = isfinite(amps) ? constrain(amps, 0.0f, 327.0f) : 0.0f;
    percent = constrain(percent, 0, 100);
    // Keep SYS_STATUS for the control station's existing telemetry path.
    uint8_t payload[31] = {};
    putLe16(payload + 14, static_cast<uint16_t>(roundf(volts * 1000.0f)));
    putLe16(payload + 16, static_cast<uint16_t>(roundf(amps * 100.0f)));
    payload[30] = static_cast<uint8_t>(percent);
    sendMavlink(1, 124, payload, sizeof(payload), kComponentId);

    // ExpressLRS translates BATTERY_STATUS (not SYS_STATUS) to the standard
    // CRSF battery sensor EdgeTX discovers on the handset.
    uint8_t batteryPayload[36] = {};
    putLe32(batteryPayload, UINT32_MAX);      // current_consumed: unavailable
    putLe32(batteryPayload + 4, UINT32_MAX);  // energy_consumed: unavailable
    putLe16(batteryPayload + 8, INT16_MAX);   // temperature: unavailable
    putLe16(batteryPayload + 10, static_cast<uint16_t>(roundf(volts * 1000.0f)));
    for (uint8_t cell = 1; cell < 10; ++cell) {
      putLe16(batteryPayload + 10 + cell * 2, UINT16_MAX);
    }
    const int16_t centiamps = static_cast<int16_t>(roundf(amps * 100.0f));
    putLe16(batteryPayload + 30, static_cast<uint16_t>(centiamps));
    batteryPayload[32] = 0;  // Battery ID 0 is the primary battery.
    batteryPayload[33] = 0;  // MAV_BATTERY_FUNCTION_ALL.
    batteryPayload[34] = 0;  // Chemistry/type unknown.
    batteryPayload[35] = static_cast<uint8_t>(static_cast<int8_t>(percent));
    sendMavlink(147, 154, batteryPayload, sizeof(batteryPayload), kComponentId);
  }

  void sendGps(double latitude, double longitude, float speedMps,
               float courseDegrees, float altitudeMeters, uint8_t satellites,
               uint8_t fixType, float hdop, uint64_t timeUsec) {
    const bool fixValid = fixType >= 2 && isfinite(latitude) && isfinite(longitude) &&
      latitude >= -90 && latitude <= 90 && longitude >= -180 && longitude <= 180;
    if (!fixValid) fixType = 1;  // MAV_GPS_FIX_TYPE_NO_FIX.
    const int32_t latE7 = fixValid ? static_cast<int32_t>(round(latitude * 1e7)) : 0;
    const int32_t lonE7 = fixValid ? static_cast<int32_t>(round(longitude * 1e7)) : 0;

    // GPS_RAW_INT wire order: time, lat/lon/alt, eph/epv, velocity/course,
    // fix type, satellite count. eph is HDOP * 100; epv is VDOP * 100.
    uint8_t rawPayload[30] = {};
    putLe64(rawPayload, timeUsec);  // Monotonic microseconds since ESP32 boot.
    putLe32(rawPayload + 8, static_cast<uint32_t>(latE7));
    putLe32(rawPayload + 12, static_cast<uint32_t>(lonE7));
    const int32_t altitudeMm = fixValid && isfinite(altitudeMeters)
      ? static_cast<int32_t>(roundf(altitudeMeters * 1000.0f)) : 0;
    putLe32(rawPayload + 16, static_cast<uint32_t>(altitudeMm));
    putLe16(rawPayload + 20, fixValid && isfinite(hdop)
      ? static_cast<uint16_t>(constrain(roundf(hdop * 100.0f), 0.0f, 65534.0f)) : 0xFFFF);
    putLe16(rawPayload + 22, 0xFFFF);  // Vertical dilution unavailable.
    putLe16(rawPayload + 24, fixValid && isfinite(speedMps)
      ? static_cast<uint16_t>(constrain(roundf(speedMps * 100.0f), 0.0f, 65534.0f)) : 0xFFFF);
    putLe16(rawPayload + 26, fixValid && isfinite(courseDegrees)
      ? static_cast<uint16_t>(constrain(roundf(courseDegrees * 100.0f), 0.0f, 35999.0f)) : 0xFFFF);
    rawPayload[28] = fixType;
    rawPayload[29] = satellites;
    sendMavlink(24, 24, rawPayload, sizeof(rawPayload), kComponentId);
    if (!fixValid) return;

    // Keep GLOBAL_POSITION_INT available to common GCS clients.
    uint8_t positionPayload[28] = {};
    putLe32(positionPayload, millis());
    putLe32(positionPayload + 4, static_cast<uint32_t>(latE7));
    putLe32(positionPayload + 8, static_cast<uint32_t>(lonE7));
    putLe32(positionPayload + 12, static_cast<uint32_t>(altitudeMm));
    putLe32(positionPayload + 16, 0);  // Relative altitude is unavailable.
    if (fixValid && isfinite(speedMps) && isfinite(courseDegrees)) {
      const float courseRadians = courseDegrees * (PI / 180.0f);
      putLe16(positionPayload + 20, static_cast<uint16_t>(static_cast<int16_t>(constrain(
        roundf(speedMps * cosf(courseRadians) * 100.0f), -32767.0f, 32767.0f))));
      putLe16(positionPayload + 22, static_cast<uint16_t>(static_cast<int16_t>(constrain(
        roundf(speedMps * sinf(courseRadians) * 100.0f), -32767.0f, 32767.0f))));
    }
    putLe16(positionPayload + 26, fixValid && isfinite(courseDegrees)
      ? static_cast<uint16_t>(constrain(roundf(courseDegrees * 100.0f), 0.0f, 35999.0f)) : 0xFFFF);
    sendMavlink(33, 104, positionPayload, sizeof(positionPayload), kComponentId);

  }

  void sendHomePosition(double latitude, double longitude, float altitudeMeters) {
    if (!isfinite(latitude) || !isfinite(longitude) || latitude < -90 || latitude > 90 ||
        longitude < -180 || longitude > 180) return;
    // MAVLink HOME_POSITION (242), 52-byte MAVLink 1 payload. Local position
    // and approach vector are unavailable; quaternion is identity.
    uint8_t payload[52] = {};
    putLe32(payload, static_cast<uint32_t>(static_cast<int32_t>(round(latitude * 1e7))));
    putLe32(payload + 4, static_cast<uint32_t>(static_cast<int32_t>(round(longitude * 1e7))));
    putLe32(payload + 8, static_cast<uint32_t>(static_cast<int32_t>(roundf(altitudeMeters * 1000.0f))));
    const float identityQuaternionW = 1.0f;
    memcpy(payload + 24, &identityQuaternionW, sizeof(identityQuaternionW));
    sendMavlink(242, 104, payload, sizeof(payload), kComponentId);
  }

  void sendStatusText(const char* message) {
    if (!message) return;
    uint8_t payload[51] = {};
    payload[0] = 6;  // MAV_SEVERITY_INFO.
    strncpy(reinterpret_cast<char*>(payload + 1), message, 50);
    sendMavlink(253, 83, payload, sizeof(payload), kComponentId);
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
  uint32_t rcOverrideFrames_ = 0;
  uint32_t rcOverrideCrcOk_ = 0;
  uint32_t rcOverrideCrcBad_ = 0;
  uint32_t rcOverrideShort_ = 0;
  uint32_t rcOverrideTargetReject_ = 0;
  uint32_t rcOverrideChannelReject_ = 0;
  uint8_t lastRcRejectCode_ = 0;  // 0 none/accepted, 1 CRC, 2 short, 3 target, 4 channel
  uint8_t lastRcTargetSystem_ = 0;
  uint8_t lastRcTargetComponent_ = 0;
  uint8_t lastRcInvalidChannel_ = 0;
  uint16_t lastRcInvalidValue_ = 0;
  uint32_t crcErrors_ = 0;

  static void putLe16(uint8_t* out, uint16_t value) {
    out[0] = static_cast<uint8_t>(value);
    out[1] = static_cast<uint8_t>(value >> 8);
  }
  static void putLe32(uint8_t* out, uint32_t value) {
    for (uint8_t i = 0; i < 4; ++i) out[i] = static_cast<uint8_t>(value >> (8 * i));
  }
  static void putLe64(uint8_t* out, uint64_t value) {
    putLe32(out, static_cast<uint32_t>(value));
    putLe32(out + 4, static_cast<uint32_t>(value >> 32));
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
    if (messageId == 70) ++rcOverrideFrames_;
    if (extra) {
      uint16_t crc = 0xffff;
      for (uint16_t i = 1; i < headerLength + length; ++i) crc = mavCrc(crc, frame_[i]);
      crc = mavCrc(crc, extra);
      uint16_t at = headerLength + length;
      const bool crcValid = frame_[at] == uint8_t(crc) && frame_[at + 1] == uint8_t(crc >> 8);
      const uint8_t* p = frame_ + headerLength;
      if (messageId == 109) {
        if (crcValid) {
          ++validRadioStatus_;
          // MAVLink 2 omits trailing zero bytes, including fixed fields.
          const uint8_t lq = length > 4 ? p[4] : 0;
          const uint8_t rssi = length > 5 ? p[5] : 0;
          const uint8_t snr = length > 7 ? p[7] : 0;
          linkQuality_ = static_cast<uint8_t>((uint16_t(lq) * 100 + 127) / 255);
          rssiDbm_ = -static_cast<int8_t>(constrain(static_cast<int>(rssi), 0, 127));
          snrDb_ = static_cast<int8_t>(snr);
          lastLinkMs_ = millis();
        } else {
          ++crcErrors_;
        }
      } else if (messageId == 70) {
        if (!crcValid) {
          ++crcErrors_;
          ++rcOverrideCrcBad_;
          lastRcRejectCode_ = 1;
        } else {
          ++rcOverrideCrcOk_;
          lastRcInvalidChannel_ = 0;
          lastRcInvalidValue_ = 0;
          if (length < 17) {
            ++rcOverrideShort_;
            lastRcRejectCode_ = 2;
          } else {
            lastRcTargetSystem_ = p[16];
            lastRcTargetComponent_ = length > 17 ? p[17] : 0;
            if (p[16] != kSystemId ||
                (length > 17 && p[17] != 0 && p[17] != kComponentId)) {
              ++rcOverrideTargetReject_;
              lastRcRejectCode_ = 3;
            } else {
              // 0 and UINT16_MAX mean ignored channels in RC_CHANNELS_OVERRIDE.
              bool valid = true;
              for (uint8_t i = 0; i < 5; ++i) {
                uint16_t us = uint16_t(p[i * 2]) | (uint16_t(p[i * 2 + 1]) << 8);
                if (us < 900 || us > 2100) {
                  valid = false;
                  lastRcInvalidChannel_ = i + 1;
                  lastRcInvalidValue_ = us;
                  break;
                }
              }
              if (!valid) {
                ++rcOverrideChannelReject_;
                lastRcRejectCode_ = 4;
              } else {
                ++validRcFrames_;
                lastRcRejectCode_ = 0;
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
          }
        }
      }
    }
    frameLength_ = 0;
    frameExpected_ = 0;
  }
};
