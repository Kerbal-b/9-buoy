#include <TinyGPSPlus.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <math.h>
#include <driver/adc.h>
#include <driver/i2s.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ---------------------------------------------------------------------------
// Configuration / Protocol Constants
// ---------------------------------------------------------------------------

// Motor pins matched to the current ESP32 wiring plan.
const int FRONT_LEFT_DIR_PIN = 18;
const int FRONT_LEFT_PWM_PIN = 19;
const int FRONT_RIGHT_DIR_PIN = 21;
const int FRONT_RIGHT_PWM_PIN = 22;
const int REAR_DIR_PIN = 32;
const int REAR_PWM_PIN = 33;

const int CURRENT_SENSOR_PIN = 34;   // ADC1 input-only
const int BATTERY_VOLTAGE_SENSOR_PIN = 35;  // ADC1 input-only
const adc1_channel_t CURRENT_SENSOR_ADC_CHANNEL = ADC1_CHANNEL_6;   // GPIO34
const adc1_channel_t BATTERY_VOLTAGE_ADC_CHANNEL = ADC1_CHANNEL_7;   // GPIO35

// I2C sensors
const int I2C_SDA_PIN = 23; 
const int I2C_SCL_PIN = 27;
const int MPU6050_INT_PIN = -1;  // Unused in the current plan
const int RCWL_TRIG_PIN = 13;
const int RCWL_ECHO_PIN = 39;
const int DS18B20_DATA_PIN = 14; 
const int I2S_BCLK_PIN = 5;
const int I2S_WS_PIN = 4;
const int I2S_DATA_IN_PIN = 36;  // VP / GPIO36
const uint8_t MPU6050_I2C_ADDRESS = 0x68;
const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;

// UART roles
// UART0: USB programming/debug (Serial)
// UART1: GPS
// UART2: reserved for future expansion
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 17;

const long SERIAL_BAUDRATE = 115200;
const long GPS_BAUDRATE = 9600;
const char* const FIRMWARE_BANNER = "ESP32_FW_2026_05_16";
const char* const WIFI_STA_SSID = "Bouy";
const char* const WIFI_STA_PASSWORD = "SuperMonkey";
const uint16_t WIFI_TCP_PORT = 5000;
const uint16_t WIFI_UDP_PORT = 5001;
const uint16_t WIFI_AUDIO_UDP_PORT = 5002;

const float ADC_REFERENCE_VOLTAGE = 3.3;
const float ADC_COUNTS = 4095.0;
const float ACS712_SENSITIVITY_VOLTS_PER_AMP = 0.066;  // 30A module
const float BATTERY_SENSOR_MAX_VOLTAGE = 16.52f;  // Calibrated to match measured pack voltage at the ADC
const float BATTERY_FULL_VOLTAGE = 12.6;
const float BATTERY_EMPTY_VOLTAGE = 9.6;
const int CURRENT_SENSOR_SAMPLE_COUNT = 32;
const int BATTERY_VOLTAGE_SAMPLE_COUNT = 16;
const int CURRENT_SENSOR_ZERO_CALIBRATION_SAMPLES = 200;
const float CURRENT_SENSOR_NOISE_FLOOR_AMPS = 0.15;

const int MOTOR_OUTPUT_LIMIT = 255;
const int VECTOR_SCALE = 100;
const float MOTOR_OUTPUT_BOOST = 2.0f;
const bool MOTOR_DIRECTION_INVERTED = true;
const uint32_t MOTOR_RAMP_INTERVAL_MS = 20;
const int MOTOR_RAMP_UP_PER_SECOND = 120;
const int MOTOR_RAMP_DOWN_PER_SECOND = 200;

const int PWM_FREQ_HZ = 20000;
const int PWM_RES_BITS = 8;
const int REAR_PWM_CHANNEL = 0;
const int FRONT_LEFT_PWM_CHANNEL = 1;
const int FRONT_RIGHT_PWM_CHANNEL = 2;
const uint32_t LED_BLINK_DISCONNECTED_MS = 800;
const uint32_t LED_FLASH_COMMAND_MS = 120;
const uint32_t IMU_STREAM_INTERVAL_MS = 50;   // 20 Hz
const uint32_t RANGE_STREAM_INTERVAL_MS = 250;
const uint32_t WATER_TEMP_STREAM_INTERVAL_MS = 2000;
const uint32_t POWER_STREAM_INTERVAL_MS = 500;
const uint32_t STATE_STREAM_INTERVAL_MS = 500;
const uint32_t GPS_STREAM_INTERVAL_MS = 500;
const uint32_t AUDIO_STREAM_INTERVAL_MS = 20;
const size_t TCP_COMMAND_BUFFER_LIMIT = 96;
const i2s_port_t AUDIO_I2S_PORT = I2S_NUM_0;
const uint32_t AUDIO_SAMPLE_RATE = 16000;
const size_t AUDIO_FRAME_SAMPLES = 256;
const int AUDIO_PCM_SHIFT = 11;
const float AUDIO_INPUT_GAIN = 4.0f;

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------

struct MotorChannel {
  int speedPin;
  int directionPin;
  int pwmChannel;
  float mixTurnGain;
  float mixThrustGain;
  float mixBias;
  bool directionInverted;
  int commandDeadband;
  int startBoostPwmForward;
  int startBoostPwmReverse;
  int startBoostMsForward;
  int startBoostMsReverse;
  int minStartPwmForward;
  int minStartPwmReverse;
  int maxPwmForward;
  int maxPwmReverse;
  float pwmCurveForward;
  float pwmCurveReverse;
  float pwmScaleForward;
  float pwmScaleReverse;
  int pwmOffsetForward;
  int pwmOffsetReverse;
  int rampUpPerSecond;
  int rampDownPerSecond;
};

struct ImuReadings {
  float accelXG;
  float accelYG;
  float accelZG;
  float gyroXDps;
  float gyroYDps;
  float gyroZDps;
  float temperatureC;
};

enum ControlMode {
  CONTROL_MODE_IDLE,
  CONTROL_MODE_MANUAL,
  CONTROL_MODE_HOLD,
  CONTROL_MODE_GOTO,
};

enum PacketType : uint8_t {
  PACKET_TYPE_IMU = 1,
  PACKET_TYPE_RANGE = 2,
  PACKET_TYPE_POWER = 3,
  PACKET_TYPE_STATE = 4,
  PACKET_TYPE_GPS = 5,
  PACKET_TYPE_AUDIO = 10,
};

struct __attribute__((packed)) PacketHeader {
  uint8_t version;
  uint8_t packetType;
  uint16_t sequence;
  uint32_t timestampMs;
};

struct __attribute__((packed)) ImuPacketPayload {
  int16_t axMg;
  int16_t ayMg;
  int16_t azMg;
  int16_t gxDpsX10;
  int16_t gyDpsX10;
  int16_t gzDpsX10;
  int16_t tempCX100;
};

struct __attribute__((packed)) PowerPacketPayload {
  uint16_t batteryMv;
  int16_t currentMa;
  uint8_t batteryPct;
  uint8_t reserved;
};

struct __attribute__((packed)) StatePacketPayload {
  uint8_t controlMode;
  uint8_t holdEnabled;
  uint8_t gpsValid;
  uint8_t motorsEnabled;
};

struct __attribute__((packed)) GpsPacketPayload {
  int32_t latE7;
  int32_t lonE7;
};

struct __attribute__((packed)) RangePacketPayload {
  uint16_t distanceMm;
  uint8_t valid;
  uint8_t reserved;
};

struct __attribute__((packed)) AudioPacketHeader {
  uint8_t version;
  uint8_t packetType;
  uint16_t sequence;
  uint32_t timestampMs;
  uint16_t sampleCount;
  uint16_t channels;
};

// ---------------------------------------------------------------------------
// Hardware Configuration
// ---------------------------------------------------------------------------

MotorChannel motors[] = {
  // Rear motor is the first tuning target. Adjust these values before
  // changing the front motors if the drivetrain is being balanced.
  {REAR_PWM_PIN, REAR_DIR_PIN, REAR_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, false, 10, 150, 96, 260, 260, 150, 76, 255, 255, 2.00f, 1.25f, 1.00f, 1.00f, 0, 0, 10, 180},
  {FRONT_LEFT_PWM_PIN, FRONT_LEFT_DIR_PIN, FRONT_LEFT_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, true, 10, 92, 92, 240, 240, 70, 70, 255, 255, 1.20f, 1.20f, 1.00f, 1.00f, 0, 0, 120, 190},
  {FRONT_RIGHT_PWM_PIN, FRONT_RIGHT_DIR_PIN, FRONT_RIGHT_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, false, 10, 150, 150, 260, 260, 150, 150, 255, 255, 2.00f, 2.00f, 1.00f, 1.00f, 0, 0, 10, 10},
};

const int MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

const float rear_axis[2] = {0.0, 1.0};
const float front_left_axis[2] = {-0.86602540378, -0.5};
const float front_right_axis[2] = {0.86602540378, -0.5};
const float* motor_axes[3] = {rear_axis, front_left_axis, front_right_axis};

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------

HardwareSerial gpsSerial(1);   // UART1
TinyGPSPlus gps;

double gpsLatitude = 0.0;
double gpsLongitude = 0.0;

int driveValues[] = {0, 0, 0};
int targetDriveValues[] = {0, 0, 0};
bool motorPwmAttached[] = {false, false, false};
bool motorStartBoostPending[] = {false, false, false};
uint32_t motorStartBoostUntilMs[] = {0, 0, 0};
ControlMode controlMode = CONTROL_MODE_IDLE;
bool holdPositionEnabled = false;
bool targetLocationSet = false;
float targetLatitude = 0.0;
float targetLongitude = 0.0;

float currentSensorZeroVoltage = ADC_REFERENCE_VOLTAGE / 2.0;
float currentSensorZeroCounts = ADC_COUNTS / 2.0;
bool mpu6050Available = false;
bool audioInputAvailable = false;
WiFiServer tcpServer(WIFI_TCP_PORT);
WiFiClient tcpClient;
WiFiUDP udpTelemetry;
IPAddress udpRemoteIp(0, 0, 0, 0);
String tcpCommandBuffer;
uint16_t udpPacketSequence = 0;
bool ledState = false;
uint32_t lastLedToggleMs = 0;
uint32_t commandFlashUntilMs = 0;
uint32_t lastImuStreamMs = 0;
uint32_t lastRangeStreamMs = 0;
uint32_t lastWaterTempStreamMs = 0;
uint32_t lastPowerStreamMs = 0;
uint32_t lastStateStreamMs = 0;
uint32_t lastGpsStreamMs = 0;
uint32_t lastAudioStreamMs = 0;
uint32_t lastDs18b20RequestMs = 0;
uint32_t lastSerialSensorTestMs = 0;
uint32_t lastMotorRampUpdateMs = 0;
int32_t rawAudioFrameBuffer[AUDIO_FRAME_SAMPLES * 2];
int16_t pcmAudioFrameBuffer[AUDIO_FRAME_SAMPLES * 2];
bool ds18b20Available = false;
bool ds18b20HasReading = false;
float ds18b20TemperatureC = 0.0f;
OneWire ds18b20OneWire(DS18B20_DATA_PIN);
DallasTemperature ds18b20Sensors(&ds18b20OneWire);

bool readMpu6050(ImuReadings& readings);
bool setupMpu6050();
bool setupDs18b20();
void updateDs18b20Sensor(uint32_t nowMs);
void emitWaterTemperatureTelemetry();
void emitSerialSensorDiagnostics();
void setupAdcSensors();
void setupRcwl1655();
bool readRcwl1655DistanceMm(uint16_t& distanceMm);
bool setupAudioInput();
bool readAudioFrame(size_t& frameCount);
void handleControlCommand(String command);
void processTcpControl();
void setupWifiControl();
void sendUdpImuTelemetry();
void sendUdpRangeTelemetry();
void sendUdpPowerTelemetry();
void sendUdpStateTelemetry();
void sendUdpGpsTelemetry();
void sendUdpAudioTelemetry();

bool controlClientConnected() {
  return tcpClient && tcpClient.connected();
}

float clampf(float val, float minv, float maxv) {
  return max(minv, min(maxv, val));
}

void emitLine(const String& line) {
  Serial.println(line);
  if (controlClientConnected()) {
    tcpClient.print(line);
    tcpClient.print("\n");
  }
}

void flashLedOnCommand() {
  commandFlashUntilMs = millis() + LED_FLASH_COMMAND_MS;
}

void updateStatusLed() {
  uint32_t nowMs = millis();

  if (nowMs < commandFlashUntilMs) {
    if (!ledState) {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    return;
  }

  if (controlClientConnected()) {
    if (!ledState) {
      ledState = true;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    return;
  }

  if (nowMs - lastLedToggleMs >= LED_BLINK_DISCONNECTED_MS) {
    lastLedToggleMs = nowMs;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }
}

template <typename T>
void sendUdpPacket(PacketType packetType, const T& payload) {
  if (!controlClientConnected()) {
    return;
  }

  PacketHeader header = {
    1,
    static_cast<uint8_t>(packetType),
    udpPacketSequence++,
    millis(),
  };

  udpRemoteIp = tcpClient.remoteIP();
  udpTelemetry.beginPacket(udpRemoteIp, WIFI_UDP_PORT);
  udpTelemetry.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  udpTelemetry.write(reinterpret_cast<const uint8_t*>(&payload), sizeof(payload));
  udpTelemetry.endPacket();
}

void sendUdpAudioPacket(const int16_t* samples, uint16_t frameCount) {
  if (!controlClientConnected()) {
    return;
  }

  AudioPacketHeader header = {
    1,
    static_cast<uint8_t>(PACKET_TYPE_AUDIO),
    udpPacketSequence++,
    millis(),
    frameCount,
    2,
  };

  udpRemoteIp = tcpClient.remoteIP();
  udpTelemetry.beginPacket(udpRemoteIp, WIFI_AUDIO_UDP_PORT);
  udpTelemetry.write(reinterpret_cast<const uint8_t*>(&header), sizeof(header));
  udpTelemetry.write(reinterpret_cast<const uint8_t*>(samples), frameCount * 2 * sizeof(int16_t));
  udpTelemetry.endPacket();
}

uint8_t encodeControlMode(ControlMode mode) {
  switch (mode) {
    case CONTROL_MODE_MANUAL:
      return 1;
    case CONTROL_MODE_HOLD:
      return 2;
    case CONTROL_MODE_GOTO:
      return 3;
    case CONTROL_MODE_IDLE:
    default:
      return 0;
  }
}

bool motorsEnabled() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (driveValues[i] != 0) {
      return true;
    }
  }
  return false;
}

void sendStatusSnapshot() {
  String modeLine = "TEL STATUS MODE ";
  switch (controlMode) {
    case CONTROL_MODE_MANUAL:
      modeLine += "MANUAL";
      break;
    case CONTROL_MODE_HOLD:
      modeLine += "HOLD";
      break;
    case CONTROL_MODE_GOTO:
      modeLine += "GOTO";
      break;
    case CONTROL_MODE_IDLE:
    default:
      modeLine += "IDLE";
      break;
  }
  emitLine(modeLine);

  if (gps.location.isValid()) {
    emitLine("TEL STATUS POS " + String(gps.location.lat(), 6) + " " + String(gps.location.lng(), 6));
  } else {
    emitLine("TEL STATUS POS UNKNOWN UNKNOWN");
  }

  if (targetLocationSet) {
    emitLine("TEL STATUS TARGET " + String(targetLatitude, 6) + " " + String(targetLongitude, 6));
  } else {
    emitLine("TEL STATUS TARGET UNKNOWN UNKNOWN");
  }

  emitLine(holdPositionEnabled ? "TEL STATUS HOLD ON" : "TEL STATUS HOLD OFF");

  float batteryVolts = readBatterySystemVoltage();
  float currentAmps = readCurrentSensorAmps();
  int batteryPct = estimateBatteryPercent(batteryVolts);

  emitLine("TEL STATUS BATTERY " + String(batteryVolts, 2) + " " + String(currentAmps, 2) + " " + String(batteryPct));
  emitLine("TEL STATUS CURRENT " + String(currentAmps, 2));

  emitWaterTemperatureTelemetry();
  emitLine("TEL SCI AIR_TEMP UNKNOWN");

  uint16_t depthMm = 0;
  if (readRcwl1655DistanceMm(depthMm)) {
    emitLine("TEL SCI DEPTH " + String(depthMm / 1000.0f, 3));
  } else {
    emitLine("TEL SCI DEPTH UNKNOWN");
  }

  ImuReadings imuReadings;
  if (readMpu6050(imuReadings)) {
    emitLine("TEL SCI IMU_ACCEL " + String(imuReadings.accelXG, 3) + " " + String(imuReadings.accelYG, 3) + " " + String(imuReadings.accelZG, 3));
    emitLine("TEL SCI IMU_GYRO " + String(imuReadings.gyroXDps, 3) + " " + String(imuReadings.gyroYDps, 3) + " " + String(imuReadings.gyroZDps, 3));
    emitLine("TEL SCI IMU_TEMP " + String(imuReadings.temperatureC, 2));
  } else {
    emitLine("TEL SCI IMU_ACCEL UNKNOWN UNKNOWN UNKNOWN");
    emitLine("TEL SCI IMU_GYRO UNKNOWN UNKNOWN UNKNOWN");
    emitLine("TEL SCI IMU_TEMP UNKNOWN");
  }
}

bool setupDs18b20() {
  pinMode(DS18B20_DATA_PIN, INPUT_PULLUP);
  ds18b20Sensors.begin();
  ds18b20Sensors.setWaitForConversion(true);
  ds18b20Sensors.setResolution(12);
  int deviceCount = ds18b20Sensors.getDeviceCount();
  ds18b20Available = (deviceCount > 0);
  ds18b20HasReading = false;
  lastDs18b20RequestMs = millis();
  return ds18b20Available;
}

void emitWaterTemperatureTelemetry() {
  if (!ds18b20HasReading) {
    emitLine("TEL SCI WATER_TEMP UNKNOWN");
    return;
  }

  emitLine("TEL SCI WATER_TEMP " + String(ds18b20TemperatureC, 2));
}

void updateDs18b20Sensor(uint32_t nowMs) {
  if (!ds18b20Available) {
    ds18b20Available = setupDs18b20();
    return;
  }

  if ((nowMs - lastDs18b20RequestMs) < WATER_TEMP_STREAM_INTERVAL_MS) {
    return;
  }

  ds18b20Sensors.requestTemperatures();
  float tempC = ds18b20Sensors.getTempCByIndex(0);
  lastDs18b20RequestMs = nowMs;

  if (tempC == DEVICE_DISCONNECTED_C) {
    ds18b20HasReading = false;
    return;
  }

  if (tempC == 85.0f) {
    ds18b20HasReading = false;
    ds18b20TemperatureC = tempC;
    return;
  }

  ds18b20TemperatureC = tempC;
  ds18b20HasReading = true;
}

void emitImuTelemetry() {
  ImuReadings imuReadings;
  if (readMpu6050(imuReadings)) {
    emitLine("TEL SCI IMU_ACCEL " + String(imuReadings.accelXG, 3) + " " + String(imuReadings.accelYG, 3) + " " + String(imuReadings.accelZG, 3));
    emitLine("TEL SCI IMU_GYRO " + String(imuReadings.gyroXDps, 3) + " " + String(imuReadings.gyroYDps, 3) + " " + String(imuReadings.gyroZDps, 3));
    emitLine("TEL SCI IMU_TEMP " + String(imuReadings.temperatureC, 2));
  } else {
    emitLine("TEL SCI IMU_ACCEL UNKNOWN UNKNOWN UNKNOWN");
    emitLine("TEL SCI IMU_GYRO UNKNOWN UNKNOWN UNKNOWN");
    emitLine("TEL SCI IMU_TEMP UNKNOWN");
  }
}

void setupRcwl1655() {
  pinMode(RCWL_TRIG_PIN, OUTPUT);
  digitalWrite(RCWL_TRIG_PIN, LOW);
  pinMode(RCWL_ECHO_PIN, INPUT);
}

bool readRcwl1655DistanceMm(uint16_t& distanceMm) {
  digitalWrite(RCWL_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(RCWL_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(RCWL_TRIG_PIN, LOW);

  const unsigned long echoTimeoutUs = 30000UL;
  unsigned long pulseWidthUs = pulseIn(RCWL_ECHO_PIN, HIGH, echoTimeoutUs);
  if (pulseWidthUs == 0) {
    return false;
  }

  float measuredMm = static_cast<float>(pulseWidthUs) * 0.1715f;
  if (measuredMm < 0.0f) {
    return false;
  }

  if (measuredMm > 65535.0f) {
    measuredMm = 65535.0f;
  }

  distanceMm = static_cast<uint16_t>(round(measuredMm));
  return true;
}

void sendUdpRangeTelemetry() {
  uint16_t distanceMm = 0;
  uint8_t valid = 0;

  if (readRcwl1655DistanceMm(distanceMm)) {
    valid = 1;
  } else {
    distanceMm = 0;
  }

  RangePacketPayload payload = {
    distanceMm,
    valid,
    0,
  };
  sendUdpPacket(PACKET_TYPE_RANGE, payload);
}

void setupWifiControl() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);

  Serial.print(F("Connecting to Wi-Fi SSID: "));
  Serial.println(WIFI_STA_SSID);

  uint32_t startMs = millis();
  const uint32_t timeoutMs = 15000;
  while (WiFi.status() != WL_CONNECTED && (millis() - startMs) < timeoutMs) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print(F("Wi-Fi connected. IP: "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("Wi-Fi connect timeout; control link unavailable"));
  }

  tcpServer.begin();
  udpTelemetry.begin(WIFI_UDP_PORT);
}

void processTcpControl() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  if (tcpClient && !tcpClient.connected()) {
    tcpClient.stop();
    tcpCommandBuffer = "";
  }

  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    if (tcpClient && tcpClient.connected()) {
      tcpClient.stop();
    }
    tcpClient = newClient;
    tcpClient.setNoDelay(true);
    udpRemoteIp = tcpClient.remoteIP();
    tcpCommandBuffer = "";
    emitLine("ACK BOOT");
  }

  while (controlClientConnected() && tcpClient.available() > 0) {
    char c = static_cast<char>(tcpClient.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      String command = tcpCommandBuffer;
      tcpCommandBuffer = "";
      command.trim();
      if (command.length() > 0) {
        flashLedOnCommand();
        handleControlCommand(command);
      }
      continue;
    }

    if (tcpCommandBuffer.length() < TCP_COMMAND_BUFFER_LIMIT) {
      tcpCommandBuffer += c;
    } else {
      tcpCommandBuffer = "";
      emitLine("ERR LINE TOO LONG");
    }
  }
}

// ---------------------------------------------------------------------------
// Motor Control
// ---------------------------------------------------------------------------

void computeMotorThrusts(float turn, float thrust, int* thrusts) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    const MotorChannel& motor = motors[i];
    float motorThrust = (2.0 / 3.0) * (
      (turn * motor_axes[i][0] * motor.mixTurnGain) +
      (thrust * motor_axes[i][1] * motor.mixThrustGain)
    );
    motorThrust += motor.mixBias;
    motorThrust = clampf(motorThrust, -1.0, 1.0);
    thrusts[i] = round(motorThrust * MOTOR_OUTPUT_LIMIT);
  }
}

int computeMotorPwm(int motorIndex, const MotorChannel& motor, int driveValue) {
  int clampedDriveValue = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  int magnitude = abs(clampedDriveValue);
  if (magnitude <= motor.commandDeadband) {
    return 0;
  }

  bool forward = clampedDriveValue > 0;
  if (motor.directionInverted) {
    forward = !forward;
  }

  int startBoostPwm = forward ? motor.startBoostPwmForward : motor.startBoostPwmReverse;
  int startBoostMs = forward ? motor.startBoostMsForward : motor.startBoostMsReverse;
  int minStartPwm = forward ? motor.minStartPwmForward : motor.minStartPwmReverse;
  int maxPwm = forward ? motor.maxPwmForward : motor.maxPwmReverse;
  float pwmCurve = forward ? motor.pwmCurveForward : motor.pwmCurveReverse;
  float pwmScale = forward ? motor.pwmScaleForward : motor.pwmScaleReverse;
  int pwmOffset = forward ? motor.pwmOffsetForward : motor.pwmOffsetReverse;

  maxPwm = constrain(maxPwm, 0, MOTOR_OUTPUT_LIMIT);
  minStartPwm = constrain(minStartPwm, 0, maxPwm);
  startBoostPwm = constrain(startBoostPwm, minStartPwm, MOTOR_OUTPUT_LIMIT);
  if (startBoostMs > 0 && motorStartBoostUntilMs[motorIndex] > millis()) {
    minStartPwm = max(minStartPwm, startBoostPwm);
  }

  float normalizedDrive = magnitude / static_cast<float>(MOTOR_OUTPUT_LIMIT);
  float curvedDrive = powf(normalizedDrive, pwmCurve);
  float pwmValue = minStartPwm + ((maxPwm - minStartPwm) * curvedDrive * pwmScale) + pwmOffset;

  int clampedPwm = round(pwmValue);
  if (clampedPwm < minStartPwm) {
    clampedPwm = minStartPwm;
  }
  if (clampedPwm > maxPwm) {
    clampedPwm = maxPwm;
  }
  return clampedPwm;
}

void applyMotorOutput(const MotorChannel& motor, int driveValue) {
  int motorIndex = -1;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (&motors[i] == &motor) {
      motorIndex = i;
      break;
    }
  }
  if (motorIndex < 0) {
    return;
  }

  int pwmValue = computeMotorPwm(motorIndex, motor, driveValue);

  if (pwmValue == 0) {
    if (motorPwmAttached[motor.pwmChannel]) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
      ledcDetach(motor.speedPin);
#else
      ledcDetachPin(motor.speedPin);
#endif
      motorPwmAttached[motor.pwmChannel] = false;
    }
    digitalWrite(motor.directionPin, LOW);
    digitalWrite(motor.speedPin, LOW);
    return;
  }

  if (!motorPwmAttached[motor.pwmChannel]) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(motor.speedPin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(motor.pwmChannel, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(motor.speedPin, motor.pwmChannel);
#endif
    motorPwmAttached[motor.pwmChannel] = true;
  }

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(motor.speedPin, pwmValue);
#else
  ledcWrite(motor.pwmChannel, pwmValue);
#endif

  // The current buoy build needs the motor polarity flipped in software.
  bool forward = driveValue > 0;
  if (motor.directionInverted) {
    forward = !forward;
  }

  // DIR pin drives one path directly and one through inverter.
  digitalWrite(motor.directionPin, forward ? HIGH : LOW);
}

void activateMotorStartBoost(int motorIndex) {
  if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
    return;
  }

  const MotorChannel& motor = motors[motorIndex];
  bool forward = driveValues[motorIndex] > 0;
  if (motor.directionInverted) {
    forward = !forward;
  }

  int startBoostMs = forward ? motor.startBoostMsForward : motor.startBoostMsReverse;
  if (startBoostMs <= 0) {
    motorStartBoostPending[motorIndex] = false;
    motorStartBoostUntilMs[motorIndex] = 0;
    return;
  }

  motorStartBoostPending[motorIndex] = false;
  motorStartBoostUntilMs[motorIndex] = millis() + static_cast<uint32_t>(startBoostMs);
}

void setMotorDrive(int motorIndex, int driveValue) {
  if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
    return;
  }

  int constrainedDrive = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  targetDriveValues[motorIndex] = constrainedDrive;
  if (constrainedDrive == 0) {
    motorStartBoostPending[motorIndex] = false;
    motorStartBoostUntilMs[motorIndex] = 0;
  } else if (driveValues[motorIndex] == 0) {
    motorStartBoostPending[motorIndex] = true;
  }
}

void applyVectorCommand(int turn, int thrust) {
  float turnf = turn / static_cast<float>(VECTOR_SCALE);
  float thrustf = thrust / static_cast<float>(VECTOR_SCALE);

  if (turn == 0 && thrust == 0) {
    stopAllMotors();
    return;
  }

  int thrusts[3];
  computeMotorThrusts(turnf, thrustf, thrusts);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, thrusts[i]);
  }
}

void stopAllMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    targetDriveValues[i] = 0;
    driveValues[i] = 0;
    motorStartBoostPending[i] = false;
    motorStartBoostUntilMs[i] = 0;
    setMotorDrive(i, 0);
    applyMotorOutput(motors[i], 0);
  }
}

void updateMotorRamps() {
  uint32_t nowMs = millis();
  if (lastMotorRampUpdateMs == 0) {
    lastMotorRampUpdateMs = nowMs;
    return;
  }

  uint32_t elapsedMs = nowMs - lastMotorRampUpdateMs;
  if (elapsedMs < MOTOR_RAMP_INTERVAL_MS) {
    return;
  }
  lastMotorRampUpdateMs = nowMs;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    int currentDrive = driveValues[i];
    int targetDrive = targetDriveValues[i];
    if (currentDrive == targetDrive) {
      continue;
    }

    int delta = targetDrive - currentDrive;
    const MotorChannel& motor = motors[i];
    int stepLimit = delta > 0
      ? max(1, static_cast<int>(round((motor.rampUpPerSecond * elapsedMs) / 1000.0f)))
      : max(1, static_cast<int>(round((motor.rampDownPerSecond * elapsedMs) / 1000.0f)));
    if (abs(delta) < stepLimit) {
      currentDrive = targetDrive;
    } else if (delta > 0) {
      currentDrive += stepLimit;
    } else {
      currentDrive -= stepLimit;
    }

    driveValues[i] = currentDrive;
    if (currentDrive != 0 && motorStartBoostPending[i]) {
      activateMotorStartBoost(i);
    }
    applyMotorOutput(motors[i], currentDrive);
  }
}

// ---------------------------------------------------------------------------
// Operational Sensors
// ---------------------------------------------------------------------------

float readCurrentSensorAverageCounts(int sampleCount) {
  long total = 0;
  for (int i = 0; i < sampleCount; i++) {
    total += adc1_get_raw(CURRENT_SENSOR_ADC_CHANNEL);
  }
  return total / static_cast<float>(sampleCount);
}

float readBatterySensorAverageCounts(int sampleCount) {
  long total = 0;
  for (int i = 0; i < sampleCount; i++) {
    total += adc1_get_raw(BATTERY_VOLTAGE_ADC_CHANNEL);
  }
  return total / static_cast<float>(sampleCount);
}

float countsToVoltage(float counts) {
  return counts * (ADC_REFERENCE_VOLTAGE / ADC_COUNTS);
}

float readBatterySystemVoltage() {
  float sensorCounts = readBatterySensorAverageCounts(BATTERY_VOLTAGE_SAMPLE_COUNT);
  return sensorCounts * (BATTERY_SENSOR_MAX_VOLTAGE / ADC_COUNTS);
}

int estimateBatteryPercent(float batteryVoltage) {
  float normalized = (batteryVoltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE);
  normalized = clampf(normalized, 0.0, 1.0);
  return round(normalized * 100.0);
}

void calibrateCurrentSensorZero() {
  currentSensorZeroCounts = readCurrentSensorAverageCounts(CURRENT_SENSOR_ZERO_CALIBRATION_SAMPLES);
  currentSensorZeroVoltage = countsToVoltage(currentSensorZeroCounts);
}

void setupAdcSensors() {
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(CURRENT_SENSOR_ADC_CHANNEL, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(BATTERY_VOLTAGE_ADC_CHANNEL, ADC_ATTEN_DB_11);
}

float readCurrentSensorAmps() {
  float sensorCounts = readCurrentSensorAverageCounts(CURRENT_SENSOR_SAMPLE_COUNT);
  float sensorVoltage = countsToVoltage(sensorCounts);
  float currentAmps = -(sensorVoltage - currentSensorZeroVoltage) / ACS712_SENSITIVITY_VOLTS_PER_AMP;

  if (fabsf(currentAmps) < CURRENT_SENSOR_NOISE_FLOOR_AMPS) {
    currentAmps = 0.0;
  }

  return currentAmps;
}

void emitSerialSensorDiagnostics() {
  pinMode(DS18B20_DATA_PIN, INPUT);

  float currentCounts = readCurrentSensorAverageCounts(CURRENT_SENSOR_SAMPLE_COUNT);
  float batteryCounts = readBatterySensorAverageCounts(BATTERY_VOLTAGE_SAMPLE_COUNT);
  float currentVoltage = countsToVoltage(currentCounts);
  float batteryVoltage = batteryCounts * (BATTERY_SENSOR_MAX_VOLTAGE / ADC_COUNTS);
  float currentAmps = -(currentVoltage - currentSensorZeroVoltage) / ACS712_SENSITIVITY_VOLTS_PER_AMP;

  Serial.println(F("---- sensor diagnostics ----"));

  Serial.print(F("ADC current raw: "));
  Serial.print(currentCounts, 1);
  Serial.print(F(" voltage: "));
  Serial.print(currentVoltage, 3);
  Serial.print(F(" current A: "));
  Serial.println(currentAmps, 3);

  Serial.print(F("ADC battery raw: "));
  Serial.print(batteryCounts, 1);
  Serial.print(F(" voltage: "));
  Serial.println(batteryVoltage, 3);
  Serial.println(F("---------------------------"));
}

bool mpu6050WriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool mpu6050ReadBytes(uint8_t startReg, uint8_t* buffer, size_t length) {
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  size_t bytesRead = Wire.requestFrom(static_cast<int>(MPU6050_I2C_ADDRESS), static_cast<int>(length), static_cast<int>(true));
  if (bytesRead != length) {
    return false;
  }

  for (size_t i = 0; i < length; i++) {
    if (Wire.available() <= 0) {
      return false;
    }
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }

  return true;
}

int16_t combineHighLow(uint8_t highByte, uint8_t lowByte) {
  return static_cast<int16_t>((static_cast<uint16_t>(highByte) << 8) | lowByte);
}

bool setupMpu6050() {
  delay(100);
  if (!mpu6050WriteRegister(MPU6050_REG_PWR_MGMT_1, 0x00)) {
    return false;
  }
  if (!mpu6050WriteRegister(MPU6050_REG_ACCEL_CONFIG, 0x00)) {
    return false;
  }
  if (!mpu6050WriteRegister(MPU6050_REG_GYRO_CONFIG, 0x00)) {
    return false;
  }
  return true;
}

bool setupAudioInput() {
  i2s_config_t config = {};
  config.mode = static_cast<i2s_mode_t>(I2S_MODE_MASTER | I2S_MODE_RX);
  config.sample_rate = AUDIO_SAMPLE_RATE;
  config.bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT;
  config.channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT;
  config.communication_format = I2S_COMM_FORMAT_STAND_I2S;
  config.intr_alloc_flags = ESP_INTR_FLAG_LEVEL1;
  config.dma_buf_count = 6;
  config.dma_buf_len = 128;
  config.use_apll = false;
  config.tx_desc_auto_clear = false;
  config.fixed_mclk = 0;

  i2s_pin_config_t pinConfig = {};
  pinConfig.bck_io_num = I2S_BCLK_PIN;
  pinConfig.ws_io_num = I2S_WS_PIN;
  pinConfig.data_out_num = I2S_PIN_NO_CHANGE;
  pinConfig.data_in_num = I2S_DATA_IN_PIN;

  esp_err_t result = i2s_driver_install(AUDIO_I2S_PORT, &config, 0, nullptr);
  if (result != ESP_OK) {
    return false;
  }

  result = i2s_set_pin(AUDIO_I2S_PORT, &pinConfig);
  if (result != ESP_OK) {
    i2s_driver_uninstall(AUDIO_I2S_PORT);
    return false;
  }

  i2s_zero_dma_buffer(AUDIO_I2S_PORT);
  return true;
}

bool readAudioFrame(size_t& frameCount) {
  if (!audioInputAvailable) {
    return false;
  }

  size_t bytesRead = 0;
  esp_err_t result = i2s_read(
    AUDIO_I2S_PORT,
    rawAudioFrameBuffer,
    sizeof(rawAudioFrameBuffer),
    &bytesRead,
    0
  );

  if (result != ESP_OK || bytesRead == 0) {
    return false;
  }

  size_t sampleCount = bytesRead / sizeof(int32_t);
  frameCount = sampleCount / 2;
  if (frameCount == 0 || frameCount > AUDIO_FRAME_SAMPLES) {
    return false;
  }

  for (size_t i = 0; i < frameCount * 2; i++) {
    int32_t sample = rawAudioFrameBuffer[i] >> AUDIO_PCM_SHIFT;
    sample = static_cast<int32_t>(sample * AUDIO_INPUT_GAIN);
    sample = constrain(sample, -32768, 32767);
    pcmAudioFrameBuffer[i] = static_cast<int16_t>(sample);
  }

  return true;
}

bool readMpu6050(ImuReadings& readings) {
  if (!mpu6050Available) {
    mpu6050Available = setupMpu6050();
    if (!mpu6050Available) {
      return false;
    }
  }

  uint8_t rawData[14];
  if (!mpu6050ReadBytes(MPU6050_REG_ACCEL_XOUT_H, rawData, sizeof(rawData))) {
    mpu6050Available = false;
    return false;
  }

  int16_t accelXRaw = combineHighLow(rawData[0], rawData[1]);
  int16_t accelYRaw = combineHighLow(rawData[2], rawData[3]);
  int16_t accelZRaw = combineHighLow(rawData[4], rawData[5]);
  int16_t tempRaw = combineHighLow(rawData[6], rawData[7]);
  int16_t gyroXRaw = combineHighLow(rawData[8], rawData[9]);
  int16_t gyroYRaw = combineHighLow(rawData[10], rawData[11]);
  int16_t gyroZRaw = combineHighLow(rawData[12], rawData[13]);

  readings.accelXG = accelXRaw / 16384.0;
  readings.accelYG = accelYRaw / 16384.0;
  readings.accelZG = accelZRaw / 16384.0;
  readings.gyroXDps = gyroXRaw / 131.0;
  readings.gyroYDps = gyroYRaw / 131.0;
  readings.gyroZDps = gyroZRaw / 131.0;
  readings.temperatureC = (tempRaw / 340.0) + 36.53;
  return true;
}

void sendUdpImuTelemetry() {
  ImuReadings imuReadings;
  if (!readMpu6050(imuReadings)) {
    return;
  }

  ImuPacketPayload payload = {
    static_cast<int16_t>(round(imuReadings.accelXG * 1000.0f)),
    static_cast<int16_t>(round(imuReadings.accelYG * 1000.0f)),
    static_cast<int16_t>(round(imuReadings.accelZG * 1000.0f)),
    static_cast<int16_t>(round(imuReadings.gyroXDps * 10.0f)),
    static_cast<int16_t>(round(imuReadings.gyroYDps * 10.0f)),
    static_cast<int16_t>(round(imuReadings.gyroZDps * 10.0f)),
    static_cast<int16_t>(round(imuReadings.temperatureC * 100.0f)),
  };
  sendUdpPacket(PACKET_TYPE_IMU, payload);
}

void sendUdpPowerTelemetry() {
  float batteryVolts = readBatterySystemVoltage();
  float currentAmps = readCurrentSensorAmps();
  int batteryPct = estimateBatteryPercent(batteryVolts);

  PowerPacketPayload payload = {
    static_cast<uint16_t>(round(batteryVolts * 1000.0f)),
    static_cast<int16_t>(round(currentAmps * 1000.0f)),
    static_cast<uint8_t>(constrain(batteryPct, 0, 100)),
    0,
  };
  sendUdpPacket(PACKET_TYPE_POWER, payload);
}

void sendUdpStateTelemetry() {
  StatePacketPayload payload = {
    encodeControlMode(controlMode),
    static_cast<uint8_t>(holdPositionEnabled ? 1 : 0),
    static_cast<uint8_t>(gps.location.isValid() ? 1 : 0),
    static_cast<uint8_t>(motorsEnabled() ? 1 : 0),
  };
  sendUdpPacket(PACKET_TYPE_STATE, payload);
}

void sendUdpGpsTelemetry() {
  if (!gps.location.isValid()) {
    return;
  }

  GpsPacketPayload payload = {
    static_cast<int32_t>(round(gps.location.lat() * 10000000.0)),
    static_cast<int32_t>(round(gps.location.lng() * 10000000.0)),
  };
  sendUdpPacket(PACKET_TYPE_GPS, payload);
}

void sendUdpAudioTelemetry() {
  size_t frameCount = 0;
  if (!readAudioFrame(frameCount)) {
    return;
  }

  sendUdpAudioPacket(pcmAudioFrameBuffer, static_cast<uint16_t>(frameCount));
}

// ---------------------------------------------------------------------------
// Communications / Protocol
// ---------------------------------------------------------------------------

int motorIndexFromName(const String& name) {
  if (name == "REAR") {
    return 0;
  }
  if (name == "FRONT_LEFT" || name == "FRONTLEFT" || name == "LEFT") {
    return 1;
  }
  if (name == "FRONT_RIGHT" || name == "FRONTRIGHT" || name == "RIGHT") {
    return 2;
  }
  if (name == "ALL") {
    return MOTOR_COUNT;
  }
  return -1;
}

void applyDirectMotorDrive(int motorIndex, int driveValue) {
  int clampedDriveValue = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  controlMode = CONTROL_MODE_MANUAL;
  holdPositionEnabled = false;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    int appliedDrive = i == motorIndex ? clampedDriveValue : 0;
    targetDriveValues[i] = appliedDrive;
    driveValues[i] = appliedDrive;
    motorStartBoostPending[i] = false;
    motorStartBoostUntilMs[i] = 0;
    if (appliedDrive != 0) {
      activateMotorStartBoost(i);
    }
    applyMotorOutput(motors[i], appliedDrive);
  }
}

void applyDirectMotorDriveAll(int driveValue) {
  int clampedDriveValue = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  controlMode = CONTROL_MODE_MANUAL;
  holdPositionEnabled = false;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    targetDriveValues[i] = clampedDriveValue;
    driveValues[i] = clampedDriveValue;
    motorStartBoostPending[i] = false;
    motorStartBoostUntilMs[i] = 0;
    if (clampedDriveValue != 0) {
      activateMotorStartBoost(i);
    }
    applyMotorOutput(motors[i], clampedDriveValue);
  }
}

void applyMotorCalibration(
  int motorIndex,
  bool forwardDirection,
  bool directionInverted,
  int startBoostPwm,
  int startBoostMs,
  int sustainMinPwm,
  int maxPwm,
  float curve,
  float rampSeconds
) {
  if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
    return;
  }

  MotorChannel& motor = motors[motorIndex];
  motor.directionInverted = directionInverted;
  startBoostPwm = constrain(startBoostPwm, 0, MOTOR_OUTPUT_LIMIT);
  startBoostMs = constrain(startBoostMs, 0, 5000);
  sustainMinPwm = constrain(sustainMinPwm, 0, MOTOR_OUTPUT_LIMIT);
  maxPwm = constrain(maxPwm, sustainMinPwm, MOTOR_OUTPUT_LIMIT);
  curve = clampf(curve, 0.10f, 8.0f);
  rampSeconds = max(0.10f, rampSeconds);

  int rampPerSecond = max(1, static_cast<int>(round(((maxPwm - sustainMinPwm) / rampSeconds))));
  if (forwardDirection) {
    motor.startBoostPwmForward = startBoostPwm;
    motor.startBoostMsForward = startBoostMs;
    motor.minStartPwmForward = sustainMinPwm;
    motor.maxPwmForward = maxPwm;
    motor.pwmCurveForward = curve;
    motor.rampUpPerSecond = rampPerSecond;
  } else {
    motor.startBoostPwmReverse = startBoostPwm;
    motor.startBoostMsReverse = startBoostMs;
    motor.minStartPwmReverse = sustainMinPwm;
    motor.maxPwmReverse = maxPwm;
    motor.pwmCurveReverse = curve;
    motor.rampDownPerSecond = rampPerSecond;
  }
}

void emitMotorCalibrationSnapshot(int motorIndex, bool forwardDirection) {
  if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
    return;
  }

  const MotorChannel& motor = motors[motorIndex];
  const char* motorName = motorIndex == 0 ? "REAR" : (motorIndex == 1 ? "FRONT_LEFT" : "FRONT_RIGHT");
  const char* directionName = forwardDirection ? "FORWARD" : "REVERSE";
  int startBoostPwm = forwardDirection ? motor.startBoostPwmForward : motor.startBoostPwmReverse;
  int startBoostMs = forwardDirection ? motor.startBoostMsForward : motor.startBoostMsReverse;
  int sustainMinPwm = forwardDirection ? motor.minStartPwmForward : motor.minStartPwmReverse;
  int maxPwm = forwardDirection ? motor.maxPwmForward : motor.maxPwmReverse;
  float curve = forwardDirection ? motor.pwmCurveForward : motor.pwmCurveReverse;
  float rampSeconds = static_cast<float>(max(1, maxPwm - sustainMinPwm)) / static_cast<float>(max(1, forwardDirection ? motor.rampUpPerSecond : motor.rampDownPerSecond));
  emitLine(
    String("TEL MOTOR CAL ") + motorName + " " + directionName + " " +
    String(startBoostPwm) + " " + String(startBoostMs) + " " + String(sustainMinPwm) + " " +
    String(maxPwm) + " " + String(curve, 2) + " " + String(rampSeconds, 2) + " " +
    String(motor.directionInverted ? 1 : 0)
  );
}

void handleControlCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }

  if (command == "PING") {
    emitLine("ACK PING");
    sendStatusSnapshot();
    return;
  }

  if (command == "REQ STATUS ALL") {
    emitLine("ACK REQ STATUS ALL");
    sendStatusSnapshot();
    return;
  }

  if (command == "CTRL STOP" || command == "STOP") {
    stopAllMotors();
    controlMode = CONTROL_MODE_IDLE;
    holdPositionEnabled = false;
    emitLine("ACK CTRL STOP");
    return;
  }

  if (command.startsWith("CTRL MOTOR CAL ")) {
    String payload = command.substring(15);
    payload.trim();

    char motorName[20];
    char directionText[12];
    int startBoostPwm = 0;
    int startBoostMs = 0;
    int sustainMinPwm = 0;
    int maxPwm = 0;
    float curve = 0.0f;
    float rampSeconds = 0.0f;
    int directionInvertedValue = -1;
    int parsed = sscanf(payload.c_str(), "%19s %11s %d %d %d %d %f %f %d", motorName, directionText, &startBoostPwm, &startBoostMs, &sustainMinPwm, &maxPwm, &curve, &rampSeconds, &directionInvertedValue);
    if (parsed < 8) {
      emitLine("ERR BAD CTRL MOTOR CAL");
      return;
    }

    int motorIndex = motorIndexFromName(String(motorName));
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
      emitLine("ERR BAD CTRL MOTOR NAME");
      return;
    }
    if (directionInvertedValue < 0) {
      directionInvertedValue = motors[motorIndex].directionInverted ? 1 : 0;
    }

    String direction = String(directionText);
    bool forwardDirection = direction == "FORWARD" || direction == "FWD";
    bool reverseDirection = direction == "REVERSE" || direction == "REV";
    if (!forwardDirection && !reverseDirection) {
      emitLine("ERR BAD CTRL MOTOR DIR");
      return;
    }

    applyMotorCalibration(motorIndex, forwardDirection, directionInvertedValue != 0, startBoostPwm, startBoostMs, sustainMinPwm, maxPwm, curve, rampSeconds);
    emitLine(
      "ACK CTRL MOTOR CAL " + String(motorName) + " " + String(directionText) +
      " " + String(startBoostPwm) + " " + String(startBoostMs) + " " + String(sustainMinPwm) + " " +
      String(maxPwm) + " " + String(curve, 2) + " " + String(rampSeconds, 2) + " " +
      String(directionInvertedValue)
    );
    return;
  }

  if (command.startsWith("REQ MOTOR CAL ")) {
    String payload = command.substring(14);
    payload.trim();

    char motorName[20];
    char directionText[12];
    if (sscanf(payload.c_str(), "%19s %11s", motorName, directionText) != 2) {
      emitLine("ERR BAD REQ MOTOR CAL");
      return;
    }

    int motorIndex = motorIndexFromName(String(motorName));
    if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
      emitLine("ERR BAD REQ MOTOR NAME");
      return;
    }

    String direction = String(directionText);
    bool forwardDirection = direction == "FORWARD" || direction == "FWD";
    bool reverseDirection = direction == "REVERSE" || direction == "REV";
    if (!forwardDirection && !reverseDirection) {
      emitLine("ERR BAD REQ MOTOR DIR");
      return;
    }

    emitMotorCalibrationSnapshot(motorIndex, forwardDirection);
    return;
  }

  if (command.startsWith("CTRL MOTOR ")) {
    String payload = command.substring(11);
    payload.trim();
    if (payload == "STOP") {
      stopAllMotors();
      controlMode = CONTROL_MODE_IDLE;
      holdPositionEnabled = false;
      emitLine("ACK CTRL MOTOR STOP");
      return;
    }

    char motorName[20];
    int drive = 0;
    if (sscanf(payload.c_str(), "%19s %d", motorName, &drive) != 2) {
      emitLine("ERR BAD CTRL MOTOR");
      return;
    }

    String motorNameText = String(motorName);
    int motorIndex = motorIndexFromName(motorNameText);
    if (motorIndex < 0) {
      emitLine("ERR BAD CTRL MOTOR NAME");
      return;
    }

    if (motorIndex == MOTOR_COUNT) {
      if (drive == 0) {
        stopAllMotors();
        controlMode = CONTROL_MODE_IDLE;
        holdPositionEnabled = false;
        emitLine("ACK CTRL MOTOR ALL 0");
        return;
      }

      applyDirectMotorDriveAll(drive);
      emitLine("ACK CTRL MOTOR ALL " + String(drive));
      return;
    }

    applyDirectMotorDrive(motorIndex, drive);
    emitLine("ACK CTRL MOTOR " + motorNameText + " " + String(drive));
    return;
  }

  if (command == "CTRL HOLD ON") {
    holdPositionEnabled = true;
    controlMode = CONTROL_MODE_HOLD;
    emitLine("ACK CTRL HOLD ON");
    return;
  }

  if (command == "CTRL HOLD OFF") {
    holdPositionEnabled = false;
    if (controlMode == CONTROL_MODE_HOLD) {
      controlMode = CONTROL_MODE_IDLE;
    }
    emitLine("ACK CTRL HOLD OFF");
    return;
  }

  if (command.startsWith("CTRL GOTO ")) {
    float lat = 0.0;
    float lon = 0.0;
    int matched = sscanf(command.c_str(), "CTRL GOTO %f %f", &lat, &lon);
    if (matched != 2) {
      emitLine("ERR BAD CTRL GOTO");
      return;
    }

    targetLatitude = lat;
    targetLongitude = lon;
    targetLocationSet = true;
    controlMode = CONTROL_MODE_GOTO;
    emitLine("ACK " + command);
    return;
  }

  int turn = 0;
  int thrust = 0;
  int vectorMatched = sscanf(command.c_str(), "CTRL VECTOR %d %d", &turn, &thrust);
  if (vectorMatched == 2) {
    if (turn < -100 || turn > 100 || thrust < -100 || thrust > 100) {
      emitLine("ERR RANGE CTRL VECTOR");
      return;
    }

    applyVectorCommand(turn, thrust);
    controlMode = CONTROL_MODE_MANUAL;
    holdPositionEnabled = false;
    emitLine("ACK CTRL VECTOR " + String(turn) + " " + String(thrust));
    return;
  }

  if (command == "BANANA") {
    stopAllMotors();
    controlMode = CONTROL_MODE_IDLE;
    emitLine("ACK CTRL STOP");
    return;
  }

  emitLine("ERR UNKNOWN " + command);
}

void processGpsSerial() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    gpsLatitude = gps.location.lat();
    gpsLongitude = gps.location.lng();
  }
}

void setupMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    // Drive the output pins low before handing them to LEDC so the motor
    // driver never sees a floating or partially configured startup state.
    pinMode(motors[i].speedPin, OUTPUT);
    digitalWrite(motors[i].speedPin, LOW);
    pinMode(motors[i].directionPin, OUTPUT);
    digitalWrite(motors[i].directionPin, LOW);
    motorPwmAttached[i] = false;
  }
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  lastLedToggleMs = millis();
  setupMotors();
  stopAllMotors();
  setupWifiControl();
  setupAdcSensors();
  delay(250);
  calibrateCurrentSensorZero();
  setupDs18b20();
  setupRcwl1655();
  mpu6050Available = setupMpu6050();
  audioInputAvailable = setupAudioInput();

  Serial.println(FIRMWARE_BANNER);
  Serial.println(F("ESP32 buoy firmware ready"));
  Serial.print(F("Wi-Fi STA SSID: "));
  Serial.println(WIFI_STA_SSID);
  Serial.print(F("TCP server port: "));
  Serial.println(WIFI_TCP_PORT);
  Serial.print(F("UDP port: "));
  Serial.println(WIFI_UDP_PORT);
  Serial.print(F("DS18B20 data pin: "));
  Serial.println(DS18B20_DATA_PIN);
  Serial.print(F("Range trigger pin: "));
  Serial.println(RCWL_TRIG_PIN);
  Serial.print(F("Range echo pin: "));
  Serial.println(RCWL_ECHO_PIN);
  Serial.print(F("Audio UDP port: "));
  Serial.println(WIFI_AUDIO_UDP_PORT);
  Serial.print(F("Station IP: "));
  Serial.println(WiFi.localIP());
  Serial.println(mpu6050Available ? F("MPU6050 ready") : F("MPU6050 unavailable"));
  Serial.println(audioInputAvailable ? F("INMP441 audio ready") : F("INMP441 audio unavailable"));
  emitSerialSensorDiagnostics();
}

void loop() {
  processTcpControl();
  processGpsSerial();
  updateStatusLed();
  updateMotorRamps();

  uint32_t nowMs = millis();
  updateDs18b20Sensor(nowMs);
  if (controlClientConnected() && (nowMs - lastImuStreamMs >= IMU_STREAM_INTERVAL_MS)) {
    lastImuStreamMs = nowMs;
    sendUdpImuTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastRangeStreamMs >= RANGE_STREAM_INTERVAL_MS)) {
    lastRangeStreamMs = nowMs;
    sendUdpRangeTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastWaterTempStreamMs >= WATER_TEMP_STREAM_INTERVAL_MS)) {
    lastWaterTempStreamMs = nowMs;
    emitWaterTemperatureTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastPowerStreamMs >= POWER_STREAM_INTERVAL_MS)) {
    lastPowerStreamMs = nowMs;
    sendUdpPowerTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastStateStreamMs >= STATE_STREAM_INTERVAL_MS)) {
    lastStateStreamMs = nowMs;
    sendUdpStateTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastGpsStreamMs >= GPS_STREAM_INTERVAL_MS)) {
    lastGpsStreamMs = nowMs;
    sendUdpGpsTelemetry();
  }
  if (controlClientConnected() && audioInputAvailable && (nowMs - lastAudioStreamMs >= AUDIO_STREAM_INTERVAL_MS)) {
    lastAudioStreamMs = nowMs;
    sendUdpAudioTelemetry();
  }

  if (nowMs - lastSerialSensorTestMs >= 5000) {
    lastSerialSensorTestMs = nowMs;
    emitSerialSensorDiagnostics();
  }
}


