#include <Arduino.h>

// SkyGuy Nano ELRS receiver, standard (non-inverted) CRSF.
// Receiver TX -> ESP32 GPIO34 (UART2 RX)
// Receiver RX <- ESP32 GPIO25 (UART2 TX)
// Receiver 5V -> ESP32 5V; receiver GND -> common GND
constexpr int CRSF_RX_PIN = 34;
constexpr int CRSF_TX_PIN = 25;
constexpr uint32_t CRSF_BAUD = 420000;

constexpr uint8_t CRSF_ADDRESS_RECEIVER = 0xEC;
constexpr uint8_t CRSF_FRAMETYPE_GPS = 0x02;
constexpr uint8_t CRSF_FRAMETYPE_BATTERY = 0x08;
constexpr uint8_t CRSF_FRAMETYPE_LINK_STATISTICS = 0x14;
constexpr uint8_t CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16;
constexpr uint8_t CRSF_FRAMETYPE_FLIGHT_MODE = 0x21;
constexpr size_t CRSF_MAX_FRAME_SIZE = 64;
constexpr uint32_t RC_TIMEOUT_MS = 500;

HardwareSerial crsfSerial(2);

uint8_t frameBuffer[CRSF_MAX_FRAME_SIZE];
size_t frameLength = 0;
size_t expectedFrameLength = 0;
uint16_t channels[16] = {};
uint32_t validFrames = 0;
uint32_t crcErrors = 0;
uint32_t lastRcFrameMs = 0;
uint32_t lastPrintMs = 0;
uint32_t lastBatteryTelemetryMs = 0;
uint32_t lastGpsTelemetryMs = 0;
uint32_t lastFlightModeTelemetryMs = 0;

uint8_t uplinkRssi1 = 0;
uint8_t uplinkRssi2 = 0;
uint8_t uplinkLinkQuality = 0;
int8_t uplinkSnr = 0;

uint8_t crc8DvbS2(const uint8_t *data, size_t length) {
  uint8_t crc = 0;
  while (length-- > 0) {
    crc ^= *data++;
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x80U) ? static_cast<uint8_t>((crc << 1U) ^ 0xD5U)
                          : static_cast<uint8_t>(crc << 1U);
    }
  }
  return crc;
}

void resetParser() {
  frameLength = 0;
  expectedFrameLength = 0;
}

void decodeChannels(const uint8_t *payload, size_t payloadLength) {
  if (payloadLength != 22) {
    return;
  }

  uint32_t bitBuffer = 0;
  uint8_t bitsAvailable = 0;
  size_t payloadIndex = 0;
  for (uint8_t channel = 0; channel < 16; ++channel) {
    while (bitsAvailable < 11 && payloadIndex < payloadLength) {
      bitBuffer |= static_cast<uint32_t>(payload[payloadIndex++]) << bitsAvailable;
      bitsAvailable += 8;
    }
    channels[channel] = bitBuffer & 0x07FFU;
    bitBuffer >>= 11;
    bitsAvailable -= 11;
  }

  lastRcFrameMs = millis();
}

void handleFrame(const uint8_t *frame, size_t totalLength) {
  const uint8_t lengthField = frame[1];
  const uint8_t receivedCrc = frame[totalLength - 1];
  const uint8_t calculatedCrc = crc8DvbS2(&frame[2], lengthField - 1);
  if (receivedCrc != calculatedCrc) {
    ++crcErrors;
    return;
  }

  ++validFrames;
  const uint8_t frameType = frame[2];
  const uint8_t *payload = &frame[3];
  const size_t payloadLength = lengthField - 2;

  if (frameType == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
    decodeChannels(payload, payloadLength);
  } else if (frameType == CRSF_FRAMETYPE_LINK_STATISTICS && payloadLength >= 4) {
    uplinkRssi1 = payload[0];
    uplinkRssi2 = payload[1];
    uplinkLinkQuality = payload[2];
    uplinkSnr = static_cast<int8_t>(payload[3]);
  }
}

void processCrsfInput() {
  while (crsfSerial.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(crsfSerial.read());

    if (frameLength == 0) {
      frameBuffer[frameLength++] = value;
      continue;
    }

    if (frameLength == 1) {
      if (value < 2 || value > CRSF_MAX_FRAME_SIZE - 2) {
        resetParser();
        continue;
      }
      frameBuffer[frameLength++] = value;
      expectedFrameLength = static_cast<size_t>(value) + 2;
      continue;
    }

    if (frameLength >= sizeof(frameBuffer)) {
      resetParser();
      continue;
    }

    frameBuffer[frameLength++] = value;
    if (frameLength == expectedFrameLength) {
      handleFrame(frameBuffer, frameLength);
      resetParser();
    }
  }
}

void writeBigEndian16(uint8_t *destination, uint16_t value) {
  destination[0] = static_cast<uint8_t>(value >> 8);
  destination[1] = static_cast<uint8_t>(value);
}

void writeBigEndian32Signed(uint8_t *destination, int32_t value) {
  const uint32_t unsignedValue = static_cast<uint32_t>(value);
  destination[0] = static_cast<uint8_t>(unsignedValue >> 24);
  destination[1] = static_cast<uint8_t>(unsignedValue >> 16);
  destination[2] = static_cast<uint8_t>(unsignedValue >> 8);
  destination[3] = static_cast<uint8_t>(unsignedValue);
}

void sendCrsfFrame(uint8_t frameType, const uint8_t *payload, size_t payloadLength) {
  if (payloadLength > CRSF_MAX_FRAME_SIZE - 4) {
    return;
  }

  uint8_t frame[CRSF_MAX_FRAME_SIZE];
  frame[0] = CRSF_ADDRESS_RECEIVER;
  frame[1] = static_cast<uint8_t>(payloadLength + 2);  // type + payload + CRC
  frame[2] = frameType;
  if (payloadLength > 0) {
    memcpy(&frame[3], payload, payloadLength);
  }
  frame[3 + payloadLength] = crc8DvbS2(&frame[2], payloadLength + 1);
  crsfSerial.write(frame, payloadLength + 4);
}

void sendBatteryTelemetry() {
  // Clearly identified bench values. Replace with ADS1115 measurements later.
  constexpr float voltageVolts = 12.0f;
  constexpr float currentAmps = 0.0f;
  constexpr uint32_t capacityMah = 0;
  constexpr uint8_t remainingPercent = 95;

  uint8_t payload[8] = {};
  writeBigEndian16(&payload[0], static_cast<uint16_t>(voltageVolts * 10.0f));
  writeBigEndian16(&payload[2], static_cast<uint16_t>(currentAmps * 10.0f));
  payload[4] = static_cast<uint8_t>(capacityMah >> 16);
  payload[5] = static_cast<uint8_t>(capacityMah >> 8);
  payload[6] = static_cast<uint8_t>(capacityMah);
  payload[7] = remainingPercent;
  sendCrsfFrame(CRSF_FRAMETYPE_BATTERY, payload, sizeof(payload));
}

void sendGpsTelemetry() {
  // Static bench position near Redwood Shores. Replace with live GT-U7 data later.
  constexpr int32_t latitudeDegE7 = 375313000;
  constexpr int32_t longitudeDegE7 = -1222480000;
  constexpr uint16_t groundSpeedKmhTimes10 = 0;
  constexpr uint16_t headingDegreesTimes100 = 0;
  constexpr uint16_t altitudeMetersPlus1000 = 1000;
  constexpr uint8_t satellites = 8;

  uint8_t payload[15] = {};
  writeBigEndian32Signed(&payload[0], latitudeDegE7);
  writeBigEndian32Signed(&payload[4], longitudeDegE7);
  writeBigEndian16(&payload[8], groundSpeedKmhTimes10);
  writeBigEndian16(&payload[10], headingDegreesTimes100);
  writeBigEndian16(&payload[12], altitudeMetersPlus1000);
  payload[14] = satellites;
  sendCrsfFrame(CRSF_FRAMETYPE_GPS, payload, sizeof(payload));
}

void sendFlightModeTelemetry() {
  const bool rcIsValid = lastRcFrameMs != 0 && millis() - lastRcFrameMs <= RC_TIMEOUT_MS;
  const char *mode = rcIsValid ? "ELRS TEST" : "FAILSAFE";
  sendCrsfFrame(CRSF_FRAMETYPE_FLIGHT_MODE,
                reinterpret_cast<const uint8_t *>(mode), strlen(mode) + 1);
}

void sendTelemetry() {
  const uint32_t now = millis();
  if (now - lastBatteryTelemetryMs >= 1000) {
    lastBatteryTelemetryMs = now;
    sendBatteryTelemetry();
  }
  if (now - lastGpsTelemetryMs >= 2000) {
    lastGpsTelemetryMs = now;
    sendGpsTelemetry();
  }
  if (now - lastFlightModeTelemetryMs >= 1000) {
    lastFlightModeTelemetryMs = now;
    sendFlightModeTelemetry();
  }
}

uint16_t crsfToMicroseconds(uint16_t raw) {
  const int32_t bounded = constrain(static_cast<int32_t>(raw), 172, 1811);
  return static_cast<uint16_t>(map(bounded, 172, 1811, 1000, 2000));
}

void printStatus() {
  const uint32_t now = millis();
  if (now - lastPrintMs < 250) {
    return;
  }
  lastPrintMs = now;

  const bool rcIsValid = lastRcFrameMs != 0 && now - lastRcFrameMs <= RC_TIMEOUT_MS;
  Serial.print(rcIsValid ? "RC=OK" : "RC=FAILSAFE");
  Serial.print(" age_ms=");
  Serial.print(lastRcFrameMs == 0 ? 0 : now - lastRcFrameMs);
  Serial.print(" CH1-8_us=");
  for (uint8_t channel = 0; channel < 8; ++channel) {
    if (channel != 0) {
      Serial.print(',');
    }
    Serial.print(crsfToMicroseconds(channels[channel]));
  }
  Serial.print(" LQ=");
  Serial.print(uplinkLinkQuality);
  Serial.print(" RSSI1=-");
  Serial.print(uplinkRssi1);
  Serial.print(" RSSI2=-");
  Serial.print(uplinkRssi2);
  Serial.print(" SNR=");
  Serial.print(uplinkSnr);
  Serial.print(" frames=");
  Serial.print(validFrames);
  Serial.print(" crc_errors=");
  Serial.println(crcErrors);
}

void setup() {
  Serial.begin(115200);
  delay(500);
  crsfSerial.setRxBufferSize(512);
  crsfSerial.begin(CRSF_BAUD, SERIAL_8N1, CRSF_RX_PIN, CRSF_TX_PIN);

  Serial.println();
  Serial.println("ESP32 ELRS/CRSF FULL-DUPLEX TEST");
  Serial.println("ELRS TX -> GPIO34; ELRS RX <- GPIO25; 5V and common GND");
  Serial.println("CRSF UART2: 420000 baud, 8N1, non-inverted");
  Serial.println("TX12: move controls and discover telemetry sensors in EdgeTX");
}

void loop() {
  processCrsfInput();
  sendTelemetry();
  printStatus();
}
