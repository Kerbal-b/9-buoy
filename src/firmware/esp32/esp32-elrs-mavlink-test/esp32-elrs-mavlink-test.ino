// Isolated ELRS MAVLink bench test. No propulsion or mission code.
// Receiver TX -> ESP32 GPIO34, receiver RX <- ESP32 GPIO25, common GND.
// Set the ELRS receiver UART protocol to MAVLink before running this sketch.

#include <Arduino.h>

constexpr int MAVLINK_RX_PIN = 34;
constexpr int MAVLINK_TX_PIN = 25;
constexpr uint32_t MAVLINK_BAUD = 460800;

HardwareSerial elrsSerial(2);
uint8_t sequenceNumber = 0;
uint32_t lastHeartbeatMs = 0;
uint32_t lastReportMs = 0;
uint32_t receivedBytes = 0;
uint8_t rxFrame[280];
uint16_t rxLength = 0;
uint16_t rxExpected = 0;
uint8_t linkLqRaw = 255;
uint8_t linkRssiMagnitude = 255;
int8_t linkSnr = 0;
uint32_t lastLinkStatusMs = 0;
uint8_t linkSequence = 0;
uint16_t rcChannels[16] = {};
uint32_t lastChannelsMs = 0;
uint32_t lastChannelsSentMs = 0;
uint32_t lastBankSentMs[2] = {};
bool channelBankChanged[2] = {};
uint8_t nextChannelBank = 0;

uint16_t crcAccumulate(uint16_t crc, uint8_t value) {
  uint8_t tmp = value ^ (crc & 0xFF);
  tmp ^= tmp << 4;
  return (crc >> 8) ^ (uint16_t(tmp) << 8) ^ (uint16_t(tmp) << 3) ^ (tmp >> 4);
}

void sendHeartbeat() {
  // MAVLink v1 HEARTBEAT, system 1/component 1, surface boat, standby.
  // MAV_AUTOPILOT_INVALID honestly identifies this as a bench source.
  uint8_t frame[17] = {
    0xFE, 9, sequenceNumber++, 1, 1, 0,
    0, 0, 0, 0,  // custom_mode
    11,          // MAV_TYPE_SURFACE_BOAT
    8,           // MAV_AUTOPILOT_INVALID
    0,           // base_mode
    3,           // MAV_STATE_STANDBY
    3,           // MAVLink protocol version
    0, 0         // checksum, filled below
  };
  uint16_t crc = 0xFFFF;
  for (size_t i = 1; i < 15; ++i) {
    crc = crcAccumulate(crc, frame[i]);
  }
  crc = crcAccumulate(crc, 50);  // HEARTBEAT CRC_EXTRA
  frame[15] = crc & 0xFF;
  frame[16] = crc >> 8;
  elrsSerial.write(frame, sizeof(frame));
}

// The ELRS receiver sends RADIO_STATUS toward the vehicle at 100 Hz.
// ELRS encodes uplink LQ as rssi=round(LQ*2.55), and uplink RSSI in
// remrssi as the positive magnitude of the negative dBm reading.
void acceptReceiverByte(uint8_t value) {
  if (rxLength == 0) {
    if (value != 0xFD && value != 0xFE) return;
    rxExpected = 0;
    rxFrame[rxLength++] = value;
    return;
  }
  rxFrame[rxLength++] = value;
  const bool v2 = rxFrame[0] == 0xFD;
  const uint8_t headerLength = v2 ? 10 : 6;
  if (rxLength == headerLength) {
    const uint8_t signatureLength = v2 && (rxFrame[2] & 1) ? 13 : 0;
    rxExpected = headerLength + rxFrame[1] + 2 + signatureLength;
    if (rxExpected > sizeof(rxFrame)) rxLength = 0;
  }
  if (rxExpected == 0 || rxLength < rxExpected) return;
  const uint32_t messageId = v2
      ? uint32_t(rxFrame[7]) | (uint32_t(rxFrame[8]) << 8) | (uint32_t(rxFrame[9]) << 16)
      : rxFrame[5];
  if ((messageId == 109 && rxFrame[1] >= 8) || (messageId == 70 && rxFrame[1] >= 18)) {
    const uint16_t checksumAt = headerLength + rxFrame[1];
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 1; i < checksumAt; ++i) crc = crcAccumulate(crc, rxFrame[i]);
    crc = crcAccumulate(crc, messageId == 109 ? 185 : 124);
    if (rxFrame[checksumAt] == uint8_t(crc) && rxFrame[checksumAt + 1] == uint8_t(crc >> 8)) {
      if (messageId == 109) {
        linkLqRaw = rxFrame[headerLength + 4];
        linkRssiMagnitude = rxFrame[headerLength + 5];
        linkSnr = int8_t(rxFrame[headerLength + 7]);
        lastLinkStatusMs = millis();
      } else if (rxFrame[headerLength + 16] == 1) {
        // RC_CHANNELS_OVERRIDE is addressed to this vehicle. The first 8
        // channels precede target IDs; channels 9-16 are MAVLink extensions.
        for (uint8_t channel = 0; channel < 16; ++channel) {
          const uint8_t offset = channel < 8 ? channel * 2 : 18 + (channel - 8) * 2;
          const uint16_t value = offset + 1 < rxFrame[1]
              ? uint16_t(rxFrame[headerLength + offset]) | (uint16_t(rxFrame[headerLength + offset + 1]) << 8)
              : 0xFFFF;
          if (rcChannels[channel] != value) channelBankChanged[channel / 8] = true;
          rcChannels[channel] = value;
        }
        lastChannelsMs = millis();
      }
    }
  }
  rxLength = 0;
  rxExpected = 0;
}

void sendLinkStatus() {
  // Re-publish one validated receiver sample per second. A fresh sample is
  // required; stale RF values must disappear when the receiver stops talking.
  if (lastLinkStatusMs == 0 || millis() - lastLinkStatusMs > 2000) return;
  uint8_t frame[17] = {
    0xFE, 9, linkSequence++, 1, 68, 109,
    0, 0, 0, 0,                // rxerrors, fixed: unknown to this bench sketch
    linkLqRaw, linkRssiMagnitude, 255, uint8_t(linkSnr), 255,
    0, 0
  };
  uint16_t crc = 0xFFFF;
  for (size_t i = 1; i < 15; ++i) crc = crcAccumulate(crc, frame[i]);
  crc = crcAccumulate(crc, 185);
  frame[15] = crc & 0xFF;
  frame[16] = crc >> 8;
  elrsSerial.write(frame, sizeof(frame));
}

void sendChannels() {
  const uint32_t now = millis();
  if (lastChannelsMs == 0 || now - lastChannelsMs > 1000) return;
  // A 30-byte RC_CHANNELS_RAW bank takes less airtime than a 50-byte full
  // channel snapshot. At 50Hz send at most two banks per second in total.
  if (now - lastChannelsSentMs < 500) return;
  uint8_t bank = nextChannelBank;
  if (!channelBankChanged[bank] && now - lastBankSentMs[bank] < 6000) bank ^= 1;
  if (!channelBankChanged[bank] && now - lastBankSentMs[bank] < 6000) return;
  uint8_t frame[30] = {0xFE, 22, sequenceNumber++, 1, 1, 35};
  for (uint8_t i = 0; i < 4; ++i) frame[6 + i] = uint8_t(now >> (i * 8));
  for (uint8_t channel = 0; channel < 8; ++channel) {
    const uint16_t value = rcChannels[bank * 8 + channel];
    frame[10 + channel * 2] = uint8_t(value);
    frame[11 + channel * 2] = uint8_t(value >> 8);
  }
  frame[26] = bank;
  frame[27] = 255;  // RSSI unknown here; receiver RADIO_STATUS has RF values
  uint16_t crc = 0xFFFF;
  for (size_t i = 1; i < 28; ++i) crc = crcAccumulate(crc, frame[i]);
  crc = crcAccumulate(crc, 244);  // RC_CHANNELS_RAW CRC_EXTRA
  frame[28] = crc & 0xFF;
  frame[29] = crc >> 8;
  elrsSerial.write(frame, sizeof(frame));
  lastChannelsSentMs = now;
  lastBankSentMs[bank] = now;
  channelBankChanged[bank] = false;
  nextChannelBank = bank ^ 1;
}

void setup() {
  Serial.begin(115200);
  elrsSerial.setRxBufferSize(512);
  elrsSerial.begin(MAVLINK_BAUD, SERIAL_8N1, MAVLINK_RX_PIN, MAVLINK_TX_PIN);
  Serial.println("ESP32 ELRS/MAVLink heartbeat bench test");
}

void loop() {
  while (elrsSerial.available() > 0) {
    const uint8_t value = elrsSerial.read();
    acceptReceiverByte(value);
    ++receivedBytes;
  }
  const uint32_t now = millis();
  if (now - lastHeartbeatMs >= 1000) {
    lastHeartbeatMs = now;
    sendHeartbeat();
    sendLinkStatus();
  }
  sendChannels();
  if (now - lastReportMs >= 1000) {
    lastReportMs = now;
    Serial.print("MAVLink HEARTBEAT sent; RX UART bytes/s=");
    Serial.println(receivedBytes);
    if (lastLinkStatusMs && now - lastLinkStatusMs < 2000) {
      Serial.printf("ELRS uplink LQ=%u%% RSSI=%d dBm SNR=%d dB\n",
                    (unsigned)((uint32_t(linkLqRaw) * 100 + 127) / 255),
                    -int(linkRssiMagnitude), int(linkSnr));
    } else {
      Serial.println("ELRS link status unavailable");
    }
    if (lastChannelsMs && now - lastChannelsMs < 1000) {
      Serial.print("TX12 channels:");
      for (uint8_t i = 0; i < 16; ++i) {
        Serial.printf(" CH%u=%u", unsigned(i + 1), unsigned(rcChannels[i]));
      }
      Serial.println();
    } else {
      Serial.println("TX12 channels unavailable");
    }
    receivedBytes = 0;
  }
}
