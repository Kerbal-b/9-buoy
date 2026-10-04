#include <TinyGPSPlus.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <math.h>
#include <driver/i2s.h>
#include <esp_timer.h>
#if __has_include(<esp_arduino_version.h>)
#include <esp_arduino_version.h>
#endif

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// ---------------------------------------------------------------------------
// Configuration / Protocol Constants
// ---------------------------------------------------------------------------

// Each IBT-4-style H-bridge uses two directional PWM inputs. Only one input
// may be driven at a time; both are held low for stop. The existing six motor
// GPIOs are retained so the controller harness can be rewired in place.
const int FRONT_LEFT_IN1_PIN = 22;
const int FRONT_LEFT_IN2_PIN = 21;
const int FRONT_RIGHT_IN1_PIN = 19;
const int FRONT_RIGHT_IN2_PIN = 18;
const int REAR_IN1_PIN = 33;
const int REAR_IN2_PIN = 32;

// I2C sensors
const int I2C_SDA_PIN = 23; 
const int I2C_SCL_PIN = 27;
const int MPU6050_INT_PIN = -1;  // Unused in the current plan
const int JSN_SR04T_TRIG_PIN = 13;
const int JSN_SR04T_ECHO_PIN = 39;
const int DS18B20_DATA_PIN = 14; 
const int I2S_BCLK_PIN = 5;
const int I2S_WS_PIN = 4;
const int I2S_DATA_IN_PIN = 36;  // VP / GPIO36
const uint8_t MPU6050_I2C_ADDRESS = 0x68;
const uint8_t MPU6050_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REG_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REG_ACCEL_XOUT_H = 0x3B;
const uint8_t ADS1115_I2C_ADDRESS = 0x48;
const uint8_t ADS1115_CURRENT_CHANNEL = 0;
const uint8_t ADS1115_BATTERY_CHANNEL = 1;
const uint8_t ADS1115_TDS_CHANNEL = 2;
const uint8_t ADS1115_TURBIDITY_CHANNEL = 3;
const float ADS1115_FULL_SCALE_VOLTS = 4.096f;
const float ACS712_INPUT_DIVIDER_RATIO = 0.60f;
const float TURBIDITY_INPUT_DIVIDER_RATIO = 0.60f;

// microSD uses the tested full-duplex-ELRS alternative mapping. GPIO1 is
// UART0 TX, so USB serial output must remain disabled while the card is active.
const int SD_CS_PIN = 1;
const int SD_SCK_PIN = 17;
const int SD_MOSI_PIN = 26;
const int SD_MISO_PIN = 35;
const uint32_t SD_SPI_FREQUENCY = 20000000UL;
const size_t SD_AUDIO_WRITE_BLOCK_BYTES = 4096;
const uint32_t SD_FLUSH_INTERVAL_MS = 1000;
const uint32_t SD_CARD_TELEMETRY_INTERVAL_MS = 30000;
const uint32_t SD_SCIENCE_INTERVAL_MS = 100;
const uint32_t SD_TELEMETRY_INTERVAL_MS = 100;
const uint32_t AUDIO_PRIORITY_SENSOR_INTERVAL_MS = 30000;

// UART roles
// UART0: USB programming/debug (Serial)
// UART1: GPS
// UART2: reserved for future expansion
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = -1;  // Receive-only; GPIO17 is the SD clock.

const long GPS_BAUDRATE = 9600;
const char* const FIRMWARE_BANNER = "ESP32_FW_2026_10_04_SDDELETE2";
const char* const WIFI_STA_SSID = "Bouy";
const char* const WIFI_STA_PASSWORD = "SuperMonkey";
const uint16_t WIFI_TCP_PORT = 5000;
const uint16_t WIFI_UDP_PORT = 5001;
const uint16_t WIFI_AUDIO_UDP_PORT = 5002;
const uint16_t WIFI_SD_FILE_PORT = 5003;

const float ADC_REFERENCE_VOLTAGE = 3.3;
const float ADC_COUNTS = 4095.0;
const float ACS712_SENSITIVITY_VOLTS_PER_AMP = 0.066;  // 30A module
const float BATTERY_SENSOR_MAX_VOLTAGE = 16.52f;  // Calibrated to match measured pack voltage at the ADC
const float BATTERY_FULL_VOLTAGE = 12.6;
const float BATTERY_EMPTY_VOLTAGE = 9.6;
const int CURRENT_SENSOR_SAMPLE_COUNT = 8;
const int BATTERY_VOLTAGE_SAMPLE_COUNT = 4;
const int CURRENT_SENSOR_ZERO_CALIBRATION_SAMPLES = 64;
const float CURRENT_SENSOR_NOISE_FLOOR_AMPS = 0.15;

const int MOTOR_OUTPUT_LIMIT = 255;
const int VECTOR_SCALE = 100;
const float MOTOR_OUTPUT_BOOST = 2.0f;
const bool MOTOR_DIRECTION_INVERTED = true;
const uint32_t MOTOR_RAMP_INTERVAL_MS = 20;
const int MOTOR_RAMP_UP_PER_SECOND = 120;
const int MOTOR_RAMP_DOWN_PER_SECOND = 200;
const uint32_t MOTOR_COMMAND_WATCHDOG_MS = 1000;

const int PWM_FREQ_HZ = 20000;
const int PWM_RES_BITS = 8;
const int REAR_IN1_PWM_CHANNEL = 0;
const int REAR_IN2_PWM_CHANNEL = 1;
const int FRONT_LEFT_IN1_PWM_CHANNEL = 2;
const int FRONT_LEFT_IN2_PWM_CHANNEL = 3;
const int FRONT_RIGHT_IN1_PWM_CHANNEL = 4;
const int FRONT_RIGHT_IN2_PWM_CHANNEL = 5;
const uint32_t LED_BLINK_DISCONNECTED_MS = 800;
const uint32_t LED_FLASH_COMMAND_MS = 120;
const uint32_t IMU_STREAM_INTERVAL_MS = 50;   // 20 Hz
const uint32_t RANGE_STREAM_INTERVAL_MS = 250;
const uint32_t WATER_TEMP_STREAM_INTERVAL_MS = 2000;
const uint32_t POWER_STREAM_INTERVAL_MS = 500;
const uint32_t STATE_STREAM_INTERVAL_MS = 500;
const uint32_t MOTOR_OUTPUT_STREAM_INTERVAL_MS = 100;
const uint32_t GPS_STREAM_INTERVAL_MS = 500;
const size_t TCP_COMMAND_BUFFER_LIMIT = 96;
const i2s_port_t AUDIO_I2S_PORT = I2S_NUM_0;
const uint32_t AUDIO_DEFAULT_SAMPLE_RATE = 16000;
const size_t AUDIO_FRAME_SAMPLES = 256;
const int AUDIO_PCM_SHIFT = 11;
// Leave digital headroom for the INMP441 pair. The previous 4x multiplier
// clipped most samples in real SD recordings and made stereo position sound unstable.
const float AUDIO_INPUT_GAIN = 1.0f;
const size_t AUDIO_QUEUE_FRAME_COUNT = 64;

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------

struct MotorChannel {
  int input1Pin;
  int input2Pin;
  int input1PwmChannel;
  int input2PwmChannel;
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

struct AudioFrame {
  uint64_t timestampUs;
  uint16_t frameCount;
  int16_t samples[AUDIO_FRAME_SAMPLES * 2];
};

struct __attribute__((packed)) AudioIndexRecord {
  uint64_t timestampUs;
  uint32_t firstSampleFrame;
  uint16_t frameCount;
  uint16_t reserved;
};

// ---------------------------------------------------------------------------
// Hardware Configuration
// ---------------------------------------------------------------------------

MotorChannel motors[] = {
  // Rear motor is the first tuning target. Adjust these values before
  // changing the front motors if the drivetrain is being balanced.
  {REAR_IN1_PIN, REAR_IN2_PIN, REAR_IN1_PWM_CHANNEL, REAR_IN2_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, true, 0, 171, 171, 260, 260, 130, 130, 255, 255, 2.00f, 2.00f, 1.00f, 1.00f, 0, 0, 125, 125},
  {FRONT_LEFT_IN1_PIN, FRONT_LEFT_IN2_PIN, FRONT_LEFT_IN1_PWM_CHANNEL, FRONT_LEFT_IN2_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, true, 0, 171, 171, 260, 260, 130, 130, 255, 255, 2.00f, 2.00f, 1.00f, 1.00f, 0, 0, 125, 125},
  {FRONT_RIGHT_IN1_PIN, FRONT_RIGHT_IN2_PIN, FRONT_RIGHT_IN1_PWM_CHANNEL, FRONT_RIGHT_IN2_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, false, 0, 171, 171, 260, 260, 130, 130, 255, 255, 2.00f, 2.00f, 1.00f, 1.00f, 0, 0, 125, 125},
};

const int MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

// Active external-motor hull. Keep this in sync with ACTIVE_MOTOR_MIX in
// station/geometry.py so the firmware and control-panel preview agree.
const bool USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT = true;

const float legacy_motor_axes[3][2] = {
  {0.0f, 1.0f},
  {0.86602540378f, -0.5f},
  {-0.86602540378f, -0.5f},
};
const float legacy_yaw_gains[3] = {1.0f, 1.0f, 1.0f};

// Axes describe force on the buoy, not water flow. The two front water jets
// point back-left/back-right. The rear water jet points left, so its positive
// force axis points right.
const float external_tangential_motor_axes[3][2] = {
  {1.0f, 0.0f},
  {0.5f, 0.86602540378f},
  {-0.5f, 0.86602540378f},
};
const float external_tangential_yaw_gains[3] = {1.0f, -1.0f, 1.0f};

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------

HardwareSerial gpsSerial(1);   // UART1
TinyGPSPlus gps;

double gpsLatitude = 0.0;
double gpsLongitude = 0.0;

int driveValues[] = {0, 0, 0};
int targetDriveValues[] = {0, 0, 0};
int appliedMotorPwm[] = {0, 0, 0};
bool motorStartBoostPending[] = {false, false, false};
uint32_t motorStartBoostUntilMs[] = {0, 0, 0};
ControlMode controlMode = CONTROL_MODE_IDLE;
bool holdPositionEnabled = false;
bool targetLocationSet = false;
float targetLatitude = 0.0;
float targetLongitude = 0.0;

float currentSensorZeroVoltage = ADC_REFERENCE_VOLTAGE / 2.0;
bool mpu6050Available = false;
bool audioInputAvailable = false;
volatile uint8_t audioChannelSelection = 0; // 0=both, 1=left, 2=right
bool audioRecordingEnabled = false;
bool audioStreamEnabled = false;
bool imuStreamEnabled = false;
volatile bool serviceMode = false;
volatile uint32_t audioSampleRateHz = AUDIO_DEFAULT_SAMPLE_RATE;
bool ads1115Available = false;
WiFiServer tcpServer(WIFI_TCP_PORT);
WiFiClient tcpClient;
WiFiServer sdFileServer(WIFI_SD_FILE_PORT);
WiFiClient sdFileClient;
WiFiUDP udpTelemetry;
IPAddress udpRemoteIp(0, 0, 0, 0);
String tcpCommandBuffer;
uint16_t udpPacketSequence = 0;
uint16_t udpAudioPacketSequence = 0;
bool ledState = false;
uint32_t lastLedToggleMs = 0;
uint32_t commandFlashUntilMs = 0;
uint32_t lastImuStreamMs = 0;
uint32_t lastRangeStreamMs = 0;
uint32_t lastWaterTempStreamMs = 0;
uint32_t lastPowerStreamMs = 0;
uint32_t lastStateStreamMs = 0;
uint32_t lastMotorOutputStreamMs = 0;
uint32_t lastGpsStreamMs = 0;
uint32_t lastDs18b20RequestMs = 0;
uint32_t lastSerialSensorTestMs = 0;
uint32_t lastMotorRampUpdateMs = 0;
uint32_t lastControlHeartbeatMs = 0;
bool hadControlClient = false;
QueueHandle_t audioFrameQueue = nullptr;
TaskHandle_t audioCaptureTaskHandle = nullptr;
volatile uint32_t audioFramesDropped = 0;
bool ds18b20Available = false;
bool ds18b20HasReading = false;
float ds18b20TemperatureC = 0.0f;
OneWire ds18b20OneWire(DS18B20_DATA_PIN);
DallasTemperature ds18b20Sensors(&ds18b20OneWire);
SPIClass sdSpi(VSPI);
File sdAudioFile;
File sdAudioIndexFile;
File sdScienceFile;
File sdTelemetryFile;
bool sdAuxFilesPaused = false;
File sdTransferFile;
String sdTransferRequest;
uint8_t sdTransferBuffer[4096];
size_t sdTransferBufferLength = 0;
size_t sdTransferBufferOffset = 0;
uint64_t sdTransferBytesRemaining = 0;
bool sdTransferActive = false;
uint32_t sdTransferStartedUs = 0;
uint64_t sdTransferReadUs = 0;
uint64_t sdTransferWriteUs = 0;
uint64_t sdTransferSentBytes = 0;
uint64_t sdLastTransferReadUs = 0;
uint64_t sdLastTransferWriteUs = 0;
uint64_t sdLastTransferBytes = 0;
uint32_t sdLastTransferWallUs = 0;
String sdSessionDirectory;
uint16_t sdNextSessionNumber = 0;
String sdExperimentStartUtc;
bool sdCardMounted = false;
uint8_t scienceExperimentState = 0; // 0=idle, 1=recording, 2=stopped awaiting save
bool sdRecordingAvailable = false;
uint8_t sdAudioWriteBuffer[SD_AUDIO_WRITE_BLOCK_BYTES];
size_t sdAudioWriteBufferUsed = 0;
uint64_t sdAudioBufferTimestampUs = 0;
uint32_t sdAudioBufferFirstFrame = 0;
uint32_t sdAudioFramesWritten = 0;
uint32_t sdWriteFailures = 0;
uint32_t sdMountFailures = 0;
uint32_t lastSdFlushMs = 0;
uint32_t lastSdScienceMs = 0;
uint32_t lastSdTelemetryMs = 0;
uint32_t lastSdCardTelemetryMs = 0;
uint64_t sdCardSizeBytes = 0;
uint64_t sdTotalBytes = 0;
uint64_t sdUsedBytes = 0;
uint16_t latestRangeDistanceMm = 0;
bool latestRangeValid = false;
float latestBatteryVolts = 0.0f;
float latestCurrentAmps = 0.0f;

bool readMpu6050(ImuReadings& readings);
bool setupMpu6050();
bool setupDs18b20();
void updateDs18b20Sensor(uint32_t nowMs);
void emitWaterTemperatureTelemetry();
void emitSensorDiagnostics();
bool setupAds1115();
void setupJsnSr04t();
bool readJsnSr04tDistanceMm(uint16_t& distanceMm);
bool setupAudioInput();
bool startAudioCaptureTask();
void processAudioFrames();
void handleControlCommand(String command);
void processTcpControl();
void processSdFileServer();
void setupWifiControl();
void sendUdpImuTelemetry();
void sendUdpRangeTelemetry();
void sendUdpPowerTelemetry();
void sendUdpStateTelemetry();
void sendMotorOutputTelemetry();
void sendUdpGpsTelemetry();
void sendUdpAudioPacket(const int16_t* samples, uint16_t frameCount);
bool mountSdCard();
bool startSdRecorder(const String& startUtc);
bool saveSdRecorder(const String& saveUtc);
void serviceSdRecorder(uint32_t nowMs);
void recordLogLine(const String& line);

bool controlClientConnected() {
  return tcpClient && tcpClient.connected();
}

float clampf(float val, float minv, float maxv) {
  return max(minv, min(maxv, val));
}

void writeWavUint16(File& file, uint16_t value) {
  uint8_t bytes[] = {
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
  };
  file.write(bytes, sizeof(bytes));
}

void writeWavUint32(File& file, uint32_t value) {
  uint8_t bytes[] = {
    static_cast<uint8_t>(value & 0xFF),
    static_cast<uint8_t>((value >> 8) & 0xFF),
    static_cast<uint8_t>((value >> 16) & 0xFF),
    static_cast<uint8_t>((value >> 24) & 0xFF),
  };
  file.write(bytes, sizeof(bytes));
}

void updateWavHeader() {
  if (!sdAudioFile) {
    return;
  }

  uint32_t audioBytes = sdAudioFramesWritten * 2UL * sizeof(int16_t);
  sdAudioFile.seek(0);
  sdAudioFile.write(reinterpret_cast<const uint8_t*>("RIFF"), 4);
  writeWavUint32(sdAudioFile, 36UL + audioBytes);
  sdAudioFile.write(reinterpret_cast<const uint8_t*>("WAVEfmt "), 8);
  writeWavUint32(sdAudioFile, 16);
  writeWavUint16(sdAudioFile, 1);
  writeWavUint16(sdAudioFile, 2);
  writeWavUint32(sdAudioFile, audioSampleRateHz);
  writeWavUint32(sdAudioFile, audioSampleRateHz * 2UL * sizeof(int16_t));
  writeWavUint16(sdAudioFile, 2 * sizeof(int16_t));
  writeWavUint16(sdAudioFile, 16);
  sdAudioFile.write(reinterpret_cast<const uint8_t*>("data"), 4);
  writeWavUint32(sdAudioFile, audioBytes);
  sdAudioFile.seek(44UL + audioBytes);
}

void disableSdRecorder() {
  sdRecordingAvailable = false;
  scienceExperimentState = 0;
  audioRecordingEnabled = false;
  ++sdWriteFailures;
  sdAudioWriteBufferUsed = 0;
  if (sdAudioFile) sdAudioFile.close();
  if (sdAudioIndexFile) sdAudioIndexFile.close();
  if (sdScienceFile) sdScienceFile.close();
  if (sdTelemetryFile) sdTelemetryFile.close();
}

bool flushAudioWriteBuffer() {
  if (!sdRecordingAvailable || sdAudioWriteBufferUsed == 0) {
    return sdRecordingAvailable;
  }

  size_t written = sdAudioFile.write(sdAudioWriteBuffer, sdAudioWriteBufferUsed);
  if (written != sdAudioWriteBufferUsed) {
    disableSdRecorder();
    return false;
  }
  sdAudioWriteBufferUsed = 0;
  return true;
}

bool mountSdCard() {
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  sdSpi.begin(SD_SCK_PIN, SD_MISO_PIN, SD_MOSI_PIN, SD_CS_PIN);
  if (!SD.begin(SD_CS_PIN, sdSpi, SD_SPI_FREQUENCY)) {
    ++sdMountFailures;
    sdSpi.end();
    return false;
  }
  sdCardSizeBytes = SD.cardSize();
  sdTotalBytes = SD.totalBytes();
  sdUsedBytes = SD.usedBytes();
  File root = SD.open("/");
  if (root) {
    while (true) {
      File entry = root.openNextFile();
      if (!entry) break;
      String name = String(entry.name());
      int slash = name.lastIndexOf('/');
      if (slash >= 0) name = name.substring(slash + 1);
      if (entry.isDirectory() && name.startsWith("session")) {
        String digits = "";
        for (size_t i = 7; i < name.length(); ++i) {
          char c = name.charAt(i);
          if (c < '0' || c > '9') break;
          digits += c;
        }
        if (digits.length() > 0) {
          uint16_t number = static_cast<uint16_t>(digits.toInt());
          if (number >= sdNextSessionNumber) sdNextSessionNumber = number + 1;
        }
      }
      entry.close();
    }
    root.close();
  }
  return true;
}

bool startSdRecorder(const String& startUtc) {
  if (!sdCardMounted || scienceExperimentState != 0) return false;
  sdSessionDirectory = "";
  for (uint16_t session = sdNextSessionNumber; session < 1000; ++session) {
    char number[16];
    snprintf(number, sizeof(number), "/session%03u-", session);
    String directory = String(number) + startUtc;
    if (!SD.exists(directory.c_str())) {
      if (!SD.mkdir(directory.c_str())) return false;
      sdSessionDirectory = directory;
      sdNextSessionNumber = session + 1;
      break;
    }
  }
  if (sdSessionDirectory.length() == 0) return false;

  sdAudioFramesWritten = 0;
  sdAudioWriteBufferUsed = 0;
  sdExperimentStartUtc = startUtc;
  sdAudioFile = SD.open((sdSessionDirectory + "/audio.wav").c_str(), FILE_WRITE);
  sdAudioIndexFile = SD.open((sdSessionDirectory + "/audio.idx").c_str(), FILE_WRITE);
  sdScienceFile = SD.open((sdSessionDirectory + "/science.csv").c_str(), FILE_WRITE);
  sdTelemetryFile = SD.open((sdSessionDirectory + "/telemetry.csv").c_str(), FILE_WRITE);
  if (!sdAudioFile || !sdAudioIndexFile || !sdScienceFile || !sdTelemetryFile) {
    disableSdRecorder();
    return false;
  }

  sdRecordingAvailable = true;
  scienceExperimentState = 1;
  audioRecordingEnabled = false;
  updateWavHeader();
  sdScienceFile.println(F("timestamp_us,imu_valid,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,imu_temp_c,water_temp_c,range_valid,range_mm,ads_current_v,ads_battery_v,tds_v,turbidity_probe_v"));
  sdTelemetryFile.println(F("timestamp_us,gps_valid,gps_date_ddmmyy,gps_time_hhmmsscc,latitude,longitude,control_mode,hold,motors_enabled,rear_drive,left_drive,right_drive,battery_v,current_a,audio_frames_dropped,sd_available,sd_mount_failures,sd_write_failures,sd_card_mib,sd_total_mib,sd_used_mib,sd_free_mib,sd_audio_bytes,sd_audio_buffer_bytes,sd_audio_queue_depth"));
  File manifest = SD.open((sdSessionDirectory + "/manifest.txt").c_str(), FILE_WRITE);
  if (manifest) {
    manifest.println(FIRMWARE_BANNER);
    manifest.println("start_utc=" + startUtc);
    manifest.println(F("status=recording"));
    manifest.println(F("timestamp_us is the 64-bit monotonic ESP timer; correlate it with GPS fields in telemetry.csv."));
    manifest.println(F("audio.wav: stereo signed 16-bit PCM; sample rate is stored in the WAV header."));
    manifest.println(F("audio.idx: repeated packed little-endian records: uint64 timestamp_us, uint32 first_sample_frame, uint16 frame_count, uint16 reserved."));
    manifest.println(F("science.csv: calibrated physical units where available; TDS and turbidity remain voltages pending wet calibration."));
    manifest.close();
  }
  lastSdFlushMs = millis();
  lastSdCardTelemetryMs = millis();
  return true;
}

bool saveSdRecorder(const String& saveUtc) {
  if (!sdRecordingAvailable || scienceExperimentState != 2) return false;
  if (!flushAudioWriteBuffer()) return false;
  updateWavHeader();
  sdAudioFile.flush();
  sdAudioIndexFile.flush();
  sdScienceFile.flush();
  sdTelemetryFile.flush();
  bool writeError = sdAudioFile.getWriteError() || sdAudioIndexFile.getWriteError() ||
    sdScienceFile.getWriteError() || sdTelemetryFile.getWriteError();
  sdAudioFile.close();
  sdAudioIndexFile.close();
  sdScienceFile.close();
  sdTelemetryFile.close();
  sdRecordingAvailable = false;
  scienceExperimentState = 0;
  sdAudioFramesWritten = 0;
  if (writeError) { ++sdWriteFailures; return false; }
  File manifest = SD.open((sdSessionDirectory + "/manifest.txt").c_str(), FILE_APPEND);
  if (!manifest) { ++sdWriteFailures; return false; }
  manifest.println("saved_utc=" + saveUtc);
  manifest.println(F("status=complete"));
  manifest.close();
  return true;
}

void recordLogLine(const String& line) {
  (void)line;
}

void emitLine(const String& line) {
  if (controlClientConnected()) {
    tcpClient.print(line);
    tcpClient.print("\n");
  }
}

String buildSdStatusLine() {
  uint64_t freeBytes = sdTotalBytes > sdUsedBytes ? sdTotalBytes - sdUsedBytes : 0;
  return
    "TEL STATUS SD " + String(sdCardMounted ? "READY " : "UNAVAILABLE ") +
    sdSessionDirectory + " AUDIO_DROPS " + String(audioFramesDropped) +
    " MOUNT_FAILURES " + String(sdMountFailures) +
    " WRITE_FAILURES " + String(sdWriteFailures) +
    " CARD_MIB " + String(static_cast<uint32_t>(sdCardSizeBytes / (1024ULL * 1024ULL))) +
    " FREE_MIB " + String(static_cast<uint32_t>(freeBytes / (1024ULL * 1024ULL)));
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
    2,
    static_cast<uint8_t>(PACKET_TYPE_AUDIO),
    udpAudioPacketSequence++,
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

bool motorCommandActive() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (driveValues[i] != 0 || targetDriveValues[i] != 0) {
      return true;
    }
  }
  return false;
}

void sendMotorOutputTelemetry() {
  if (!controlClientConnected()) {
    return;
  }
  tcpClient.print("TEL MOTOR OUT ");
  for (int i = 0; i < MOTOR_COUNT; i++) {
    if (i > 0) {
      tcpClient.print(" ");
    }
    tcpClient.print(driveValues[i]);
    tcpClient.print(" ");
    tcpClient.print(appliedMotorPwm[i]);
  }
  tcpClient.print("\n");
}

void sendStatusSnapshot() {
  emitLine(String("TEL STATUS FIRMWARE ") + FIRMWARE_BANNER);
  const char* audioChannelName = audioChannelSelection == 1 ? "LEFT" : (audioChannelSelection == 2 ? "RIGHT" : "BOTH");
  emitLine(
    String("TEL STATUS AUDIO ") + audioChannelName +
    " RECORDING " + (audioRecordingEnabled ? "ON" : "OFF") +
    " SD " + (sdCardMounted ? "READY" : "UNAVAILABLE") +
    " RATE " + String(audioSampleRateHz) +
    " QUALITY_LOCKED " + String(sdAudioFramesWritten > 0 ? "ON" : "OFF")
  );
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
  emitLine(buildSdStatusLine());
  emitLine(String("TEL STATUS SCIENCE ") +
    (scienceExperimentState == 1 ? "RECORDING " : (scienceExperimentState == 2 ? "STOPPED " : "IDLE ")) +
    (sdSessionDirectory.length() ? sdSessionDirectory : "NONE"));
  emitLine(String("TEL STATUS STREAM AUDIO ") + (audioStreamEnabled ? "ON" : "OFF"));
  emitLine(String("TEL STATUS STREAM IMU ") + (imuStreamEnabled ? "ON" : "OFF"));
  emitLine(String("TEL STATUS SERVICE ") + (serviceMode ? "ON" : "OFF"));

  float batteryVolts = readBatterySystemVoltage();
  float currentAmps = readCurrentSensorAmps();
  int batteryPct = estimateBatteryPercent(batteryVolts);

  emitLine("TEL STATUS BATTERY " + String(batteryVolts, 2) + " " + String(currentAmps, 2) + " " + String(batteryPct));
  emitLine("TEL STATUS CURRENT " + String(currentAmps, 2));

  emitWaterTemperatureTelemetry();
  emitLine("TEL SCI AIR_TEMP UNKNOWN");

  uint16_t depthMm = 0;
  if (readJsnSr04tDistanceMm(depthMm)) {
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
  ds18b20Sensors.setWaitForConversion(false);
  ds18b20Sensors.setResolution(12);
  int deviceCount = ds18b20Sensors.getDeviceCount();
  ds18b20Available = (deviceCount > 0);
  ds18b20HasReading = false;
  lastDs18b20RequestMs = millis();
  if (ds18b20Available) ds18b20Sensors.requestTemperatures();
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
    uint32_t retryIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : 5000UL;
    if (nowMs - lastDs18b20RequestMs >= retryIntervalMs) ds18b20Available = setupDs18b20();
    return;
  }

  uint32_t intervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : WATER_TEMP_STREAM_INTERVAL_MS;
  if ((nowMs - lastDs18b20RequestMs) < intervalMs) {
    return;
  }

  float tempC = ds18b20Sensors.getTempCByIndex(0);
  ds18b20Sensors.requestTemperatures();
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

void setupJsnSr04t() {
  pinMode(JSN_SR04T_TRIG_PIN, OUTPUT);
  digitalWrite(JSN_SR04T_TRIG_PIN, LOW);
  pinMode(JSN_SR04T_ECHO_PIN, INPUT);
}

bool readJsnSr04tDistanceMm(uint16_t& distanceMm) {
  digitalWrite(JSN_SR04T_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(JSN_SR04T_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(JSN_SR04T_TRIG_PIN, LOW);

  const unsigned long echoTimeoutUs = 30000UL;
  unsigned long pulseWidthUs = pulseIn(JSN_SR04T_ECHO_PIN, HIGH, echoTimeoutUs);
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
  RangePacketPayload payload = {
    latestRangeValid ? latestRangeDistanceMm : static_cast<uint16_t>(0),
    static_cast<uint8_t>(latestRangeValid ? 1 : 0),
    0,
  };
  sendUdpPacket(PACKET_TYPE_RANGE, payload);
}

void setupWifiControl() {
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_STA_SSID, WIFI_STA_PASSWORD);
  recordLogLine("Connecting to Wi-Fi SSID: " + String(WIFI_STA_SSID));

  tcpServer.begin();
  sdFileServer.begin();
  udpTelemetry.begin(WIFI_UDP_PORT);
}

void pauseSdAuxFiles() {
  if (!sdRecordingAvailable || sdAuxFilesPaused) return;
  sdAudioIndexFile.flush();
  sdAudioIndexFile.close();
  sdAuxFilesPaused = true;
}

void resumeSdAuxFiles() {
  if (!sdAuxFilesPaused) return;
  sdAuxFilesPaused = false;
  if (!sdRecordingAvailable) return;
  sdAudioIndexFile = SD.open((sdSessionDirectory + "/audio.idx").c_str(), FILE_APPEND);
  if (!sdAudioIndexFile) disableSdRecorder();
}

void closeSdTransferClient() {
  if (sdTransferActive) {
    sdLastTransferReadUs = sdTransferReadUs;
    sdLastTransferWriteUs = sdTransferWriteUs;
    sdLastTransferBytes = sdTransferSentBytes;
    sdLastTransferWallUs = micros() - sdTransferStartedUs;
  }
  if (sdTransferFile) sdTransferFile.close();
  resumeSdAuxFiles();
  sdTransferActive = false;
  sdTransferBufferLength = 0;
  sdTransferBufferOffset = 0;
  sdTransferBytesRemaining = 0;
  sdTransferRequest = "";
  if (sdFileClient) sdFileClient.stop();
}

String sdEntryPath(const String& parentPath, const String& entryName) {
  if (entryName.startsWith("/")) return entryName;
  if (parentPath == "/") return "/" + entryName;
  return parentPath + "/" + entryName;
}

uint64_t sdDirectoryByteSize(const String& path) {
  File directory = SD.open(path.c_str());
  if (!directory || !directory.isDirectory()) return 0;
  uint64_t total = 0;
  while (true) {
    File item = directory.openNextFile();
    if (!item) break;
    if (!item.isDirectory()) total += item.size();
    item.close();
  }
  directory.close();
  return total;
}

void sendSdDirectoryListing(File& directory, const String& directoryPath) {
  while (sdFileClient.connected()) {
    File entry = directory.openNextFile();
    if (!entry) break;
    String entryName = String(entry.name());
    String entryPath = sdEntryPath(directoryPath, entryName);
    if (entry.isDirectory()) {
      entry.close();
      sdFileClient.print("DIR|");
      sdFileClient.print(entryPath);
      sdFileClient.print("|");
      sdFileClient.print(entryName);
      sdFileClient.print("|");
      uint64_t directoryBytes = directoryPath == "/" && scienceExperimentState == 0 && entryPath.startsWith("/session")
        ? sdDirectoryByteSize(entryPath) : 0;
      sdFileClient.print(static_cast<unsigned long long>(directoryBytes));
      sdFileClient.print("\n");
    } else {
      sdFileClient.print("FILE|");
      sdFileClient.print(entryPath);
      sdFileClient.print("|");
      sdFileClient.print(entryName);
      sdFileClient.print("|");
      sdFileClient.print(static_cast<unsigned long>(entry.size()));
      sdFileClient.print("\n");
    }
    entry.close();
  }
}

bool isSafeSdPath(const String& path) {
  return path.startsWith("/") && path.indexOf("..") < 0 && path.indexOf('\\') < 0 && path.indexOf('|') < 0;
}

bool isSafeSessionDirectoryPath(const String& path) {
  if (!isSafeSdPath(path) || !path.startsWith("/session") || path.indexOf('/', 1) >= 0) return false;
  String name = path.substring(1);
  String suffix = name.substring(7);
  if (suffix.length() >= 3) {
    bool legacyNumberOnly = true;
    for (size_t i = 0; i < suffix.length(); ++i) {
      if (!isDigit(suffix.charAt(i))) {
        legacyNumberOnly = false;
        break;
      }
    }
    if (legacyNumberOnly) return true;  // legacy /sessionNNN folders
  }
  if (suffix.startsWith("-")) {
    suffix = suffix.substring(1);  // legacy /session-YYYYMMDDTHHMMSSZ
  } else {
    int separator = suffix.indexOf('-');
    if (separator < 3) return false;
    for (int i = 0; i < separator; ++i) {
      if (!isDigit(suffix.charAt(i))) return false;
    }
    suffix = suffix.substring(separator + 1);  // /sessionNNN-YYYYMMDDTHHMMSSZ
  }
  if (suffix.length() != 16 || suffix.charAt(8) != 'T' || suffix.charAt(15) != 'Z') return false;
  for (int i = 0; i < 15; ++i) {
    if (i == 8) continue;
    if (!isDigit(suffix.charAt(i))) return false;
  }
  return true;
}

bool removeSdDirectoryRecursively(const String& path) {
  File directory = SD.open(path.c_str());
  if (!directory || !directory.isDirectory()) return false;
  while (true) {
    File entry = directory.openNextFile();
    if (!entry) break;
    String entryName = String(entry.name());
    String entryPath = sdEntryPath(path, entryName);
    bool isDirectory = entry.isDirectory();
    entry.close();
    bool removed = isDirectory
      ? removeSdDirectoryRecursively(entryPath)
      : SD.remove(entryPath.c_str());
    if (!removed) {
      directory.close();
      return false;
    }
  }
  directory.close();
  return SD.rmdir(path.c_str());
}

void beginSdFileRequest(const String& request) {
  if (!sdCardMounted) {
    sdFileClient.println("ERR SD UNAVAILABLE");
    closeSdTransferClient();
    return;
  }
  if (request == "STATS") {
    sdFileClient.print("INFO|TRANSFER|");
    sdFileClient.print(static_cast<unsigned long long>(sdLastTransferBytes));
    sdFileClient.print("|");
    sdFileClient.print(static_cast<unsigned long long>(sdLastTransferReadUs));
    sdFileClient.print("|");
    sdFileClient.print(static_cast<unsigned long long>(sdLastTransferWriteUs));
    sdFileClient.print("|");
    sdFileClient.println(sdLastTransferWallUs);
    sdFileClient.println("END");
    closeSdTransferClient();
    return;
  }
  if (request == "LIST" || request.startsWith("LIST ")) {
    String directoryPath = request == "LIST" ? "/" : request.substring(5);
    directoryPath.trim();
    if (directoryPath.length() == 0) directoryPath = "/";
    if (!isSafeSdPath(directoryPath)) {
      sdFileClient.println("ERR BAD PATH");
      closeSdTransferClient();
      return;
    }
    pauseSdAuxFiles();
    if (directoryPath == "/") {
      sdCardSizeBytes = SD.cardSize();
      sdTotalBytes = SD.totalBytes();
      sdUsedBytes = SD.usedBytes();
      uint64_t freeBytes = sdTotalBytes > sdUsedBytes ? sdTotalBytes - sdUsedBytes : 0;
      sdFileClient.print("INFO|CARD|");
      sdFileClient.print(static_cast<unsigned long long>(sdCardSizeBytes));
      sdFileClient.print("|");
      sdFileClient.print(static_cast<unsigned long long>(sdTotalBytes));
      sdFileClient.print("|");
      sdFileClient.print(static_cast<unsigned long long>(sdUsedBytes));
      sdFileClient.print("|");
      sdFileClient.println(static_cast<unsigned long long>(freeBytes));
    }
    sdFileClient.println("OK LIST");
    File directory = SD.open(directoryPath.c_str());
    if (!directory || !directory.isDirectory()) {
      sdFileClient.println("ERR DIRECTORY NOT FOUND");
      closeSdTransferClient();
      return;
    }
    sendSdDirectoryListing(directory, directoryPath);
    directory.close();
    sdFileClient.println("END");
    closeSdTransferClient();
    return;
  }
  if (request.startsWith("DELETE ")) {
    String path = request.substring(7);
    path.trim();
    if (!isSafeSessionDirectoryPath(path)) {
      sdFileClient.println("ERR BAD SESSION PATH");
      closeSdTransferClient();
      return;
    }
    if (scienceExperimentState != 0) {
      sdFileClient.println("ERR CLOSE SCIENCE SESSION BEFORE DELETING");
      closeSdTransferClient();
      return;
    }
    if (!serviceMode) {
      sdFileClient.println("ERR ENTER SERVICE MODE BEFORE DELETING");
      closeSdTransferClient();
      return;
    }
    File sessionDirectory = SD.open(path.c_str());
    bool isDirectory = sessionDirectory && sessionDirectory.isDirectory();
    if (sessionDirectory) sessionDirectory.close();
    if (!isDirectory) {
      sdFileClient.println("ERR SESSION NOT FOUND");
      closeSdTransferClient();
      return;
    }
    pauseSdAuxFiles();
    if (!removeSdDirectoryRecursively(path)) {
      sdFileClient.println("ERR SESSION DELETE FAILED");
      closeSdTransferClient();
      return;
    }
    sdFileClient.println("OK DELETE");
    closeSdTransferClient();
    return;
  }
  if (!request.startsWith("GET ")) {
    sdFileClient.println("ERR UNKNOWN REQUEST");
    closeSdTransferClient();
    return;
  }

  String path = request.substring(4);
  path.trim();
  if (!isSafeSdPath(path)) {
    sdFileClient.println("ERR BAD PATH");
    closeSdTransferClient();
    return;
  }
  if (scienceExperimentState == 1) {
    sdFileClient.println("ERR STOP SCIENCE EXPERIMENT BEFORE DOWNLOADING");
    closeSdTransferClient();
    return;
  }
  if (!serviceMode) {
    sdFileClient.println("ERR ENTER SERVICE MODE BEFORE DOWNLOADING");
    closeSdTransferClient();
    return;
  }
  pauseSdAuxFiles();
  sdTransferFile = SD.open(path.c_str(), FILE_READ);
  if (!sdTransferFile || sdTransferFile.isDirectory()) {
    sdFileClient.println("ERR FILE NOT FOUND");
    closeSdTransferClient();
    return;
  }
  sdTransferBytesRemaining = sdTransferFile.size();
  sdTransferBufferLength = 0;
  sdTransferBufferOffset = 0;
  sdFileClient.print("OK FILE ");
  sdFileClient.print(static_cast<unsigned long>(sdTransferBytesRemaining));
  sdFileClient.print(" ");
  sdFileClient.println(path.substring(path.lastIndexOf('/') + 1));
  sdTransferStartedUs = micros();
  sdTransferReadUs = 0;
  sdTransferWriteUs = 0;
  sdTransferSentBytes = 0;
  sdTransferActive = true;
}

void processSdFileServer() {
  if (sdTransferActive && !serviceMode) {
    closeSdTransferClient();
    return;
  }
  if (WiFi.status() != WL_CONNECTED) {
    serviceMode = false;
    audioStreamEnabled = false;
    imuStreamEnabled = false;
    closeSdTransferClient();
    return;
  }
  if (!sdFileClient || !sdFileClient.connected()) {
    if (sdFileClient) closeSdTransferClient();
    WiFiClient incoming = sdFileServer.available();
    if (!incoming) return;
    sdFileClient = incoming;
    sdFileClient.setNoDelay(false);
    sdTransferRequest = "";
    sdTransferActive = false;
  }

  if (!sdTransferActive) {
    while (sdFileClient.available() > 0) {
      char c = static_cast<char>(sdFileClient.read());
      if (c == '\r') continue;
      if (c == '\n') {
        String request = sdTransferRequest;
        sdTransferRequest = "";
        request.trim();
        beginSdFileRequest(request);
        return;
      }
      if (sdTransferRequest.length() < 220) sdTransferRequest += c;
      else {
        sdFileClient.println("ERR REQUEST TOO LONG");
        closeSdTransferClient();
        return;
      }
    }
    return;
  }

  for (uint8_t chunk = 0; chunk < 4; ++chunk) {
    if (sdTransferBufferOffset < sdTransferBufferLength) {
      uint32_t startedUs = micros();
      size_t written = sdFileClient.write(
        sdTransferBuffer + sdTransferBufferOffset,
        sdTransferBufferLength - sdTransferBufferOffset
      );
      sdTransferWriteUs += micros() - startedUs;
      sdTransferSentBytes += written;
      if (written == 0) return;
      sdTransferBufferOffset += written;
      if (sdTransferBufferOffset < sdTransferBufferLength) return;
      sdTransferBufferOffset = 0;
      sdTransferBufferLength = 0;
    }
    if (sdTransferBytesRemaining == 0) {
      closeSdTransferClient();
      return;
    }
    size_t bytesToRead = sdTransferBytesRemaining < sizeof(sdTransferBuffer)
      ? static_cast<size_t>(sdTransferBytesRemaining)
      : sizeof(sdTransferBuffer);
    uint32_t startedUs = micros();
    int bytesRead = sdTransferFile.read(sdTransferBuffer, bytesToRead);
    sdTransferReadUs += micros() - startedUs;
    if (bytesRead <= 0) {
      closeSdTransferClient();
      return;
    }
    sdTransferBufferLength = static_cast<size_t>(bytesRead);
    sdTransferBufferOffset = 0;
    sdTransferBytesRemaining -= static_cast<uint64_t>(bytesRead);
  }
}

void markControlHeartbeat() {
  lastControlHeartbeatMs = millis();
}

void stopMotorsForSafety(const __FlashStringHelper* reason) {
  bool wasActive = motorCommandActive();
  stopAllMotors();
  controlMode = CONTROL_MODE_IDLE;
  holdPositionEnabled = false;
  if (wasActive) {
    recordLogLine("SAFETY MOTOR STOP: " + String(reason));
  }
}

void updateMotorCommandWatchdog() {
  if (!motorCommandActive()) {
    return;
  }

  if (!controlClientConnected()) {
    stopMotorsForSafety(F("CONTROL LINK LOST"));
    return;
  }

  uint32_t nowMs = millis();
  if (lastControlHeartbeatMs == 0 || nowMs - lastControlHeartbeatMs > MOTOR_COMMAND_WATCHDOG_MS) {
    stopMotorsForSafety(F("COMMAND WATCHDOG TIMEOUT"));
  }
}

void processTcpControl() {
  if (WiFi.status() != WL_CONNECTED) {
    if (hadControlClient || motorCommandActive()) {
      stopMotorsForSafety(F("WI-FI LOST"));
    }
    hadControlClient = false;
    if (tcpClient) {
      tcpClient.stop();
    }
    tcpCommandBuffer = "";
    return;
  }

  if (hadControlClient && !controlClientConnected()) {
    serviceMode = false;
    audioStreamEnabled = false;
    imuStreamEnabled = false;
    stopMotorsForSafety(F("TCP CLIENT DISCONNECTED"));
    tcpClient.stop();
    tcpCommandBuffer = "";
    hadControlClient = false;
  }

  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    serviceMode = false;
    audioStreamEnabled = false;
    imuStreamEnabled = false;
    if (tcpClient && tcpClient.connected()) {
      stopMotorsForSafety(F("CONTROL CLIENT REPLACED"));
      tcpClient.stop();
    }
    tcpClient = newClient;
    tcpClient.setNoDelay(true);
    udpRemoteIp = tcpClient.remoteIP();
    tcpCommandBuffer = "";
    hadControlClient = true;
    markControlHeartbeat();
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

void computeMotorThrusts(float lateral, float thrust, float yaw, int* thrusts) {
  float mixedThrusts[3];
  float maxMagnitude = 0.0f;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    const MotorChannel& motor = motors[i];
    const float axisX = USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT
      ? external_tangential_motor_axes[i][0]
      : legacy_motor_axes[i][0];
    const float axisY = USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT
      ? external_tangential_motor_axes[i][1]
      : legacy_motor_axes[i][1];
    const float yawGain = USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT
      ? external_tangential_yaw_gains[i]
      : legacy_yaw_gains[i];
    float motorThrust = (2.0 / 3.0) * (
      (lateral * axisX * motor.mixTurnGain) +
      (thrust * axisY * motor.mixThrustGain)
    ) * MOTOR_OUTPUT_BOOST;
    motorThrust += yaw * yawGain;
    motorThrust += motor.mixBias;
    mixedThrusts[i] = motorThrust;
    maxMagnitude = max(maxMagnitude, abs(motorThrust));
  }

  float outputScale = (yaw != 0.0f && maxMagnitude > 1.0f) ? (1.0f / maxMagnitude) : 1.0f;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    float motorThrust = clampf(mixedThrusts[i] * outputScale, -1.0f, 1.0f);
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

void writeMotorPwm(int pin, int pwmChannel, int pwmValue) {
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
  ledcWrite(pin, pwmValue);
#else
  ledcWrite(pwmChannel, pwmValue);
#endif
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
  appliedMotorPwm[motorIndex] = pwmValue;

  // Always remove drive from both bridge inputs before selecting a direction.
  // This guarantees a safe LOW/LOW transition instead of briefly commanding
  // both sides of the H-bridge during a sign change.
  writeMotorPwm(motor.input1Pin, motor.input1PwmChannel, 0);
  writeMotorPwm(motor.input2Pin, motor.input2PwmChannel, 0);

  if (pwmValue == 0) {
    return;
  }

  // Motor polarity remains a per-motor calibration so the established thrust
  // directions survive the controller replacement.
  bool forward = driveValue > 0;
  if (motor.directionInverted) {
    forward = !forward;
  }

  if (forward) {
    writeMotorPwm(motor.input1Pin, motor.input1PwmChannel, pwmValue);
  } else {
    writeMotorPwm(motor.input2Pin, motor.input2PwmChannel, pwmValue);
  }
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
  } else if (
    driveValues[motorIndex] == 0 ||
    ((driveValues[motorIndex] > 0) != (constrainedDrive > 0))
  ) {
    motorStartBoostPending[motorIndex] = true;
    motorStartBoostUntilMs[motorIndex] = 0;
  }
}

void applyMotionCommand(int lateral, int thrust, int yaw) {
  float lateralf = lateral / static_cast<float>(VECTOR_SCALE);
  float thrustf = thrust / static_cast<float>(VECTOR_SCALE);
  float yawf = yaw / static_cast<float>(VECTOR_SCALE);

  if (lateral == 0 && thrust == 0 && yaw == 0) {
    stopAllMotors();
    return;
  }

  int thrusts[3];
  computeMotorThrusts(lateralf, thrustf, yawf, thrusts);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, thrusts[i]);
  }
}

void applyVectorCommand(int turn, int thrust) {
  applyMotionCommand(turn, thrust, 0);
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
      // Re-apply the steady-state curve when a short move reaches its target
      // before the start boost expires. Otherwise the last boost PWM can stay
      // latched indefinitely even though the configured boost time elapsed.
      if (motorStartBoostUntilMs[i] != 0 && nowMs >= motorStartBoostUntilMs[i]) {
        motorStartBoostUntilMs[i] = 0;
        applyMotorOutput(motors[i], currentDrive);
      }
      continue;
    }

    int delta = targetDrive - currentDrive;
    const MotorChannel& motor = motors[i];
    int stepLimit = delta > 0
      ? max(1, static_cast<int>(round((motor.rampUpPerSecond * elapsedMs) / 1000.0f)))
      : max(1, static_cast<int>(round((motor.rampDownPerSecond * elapsedMs) / 1000.0f)));
    bool reversing = currentDrive != 0 && targetDrive != 0 && ((currentDrive > 0) != (targetDrive > 0));
    if (reversing && abs(currentDrive) <= stepLimit) {
      // Hold both H-bridge inputs low for at least one ramp interval before
      // applying PWM to the opposite input.
      currentDrive = 0;
    } else if (abs(delta) < stepLimit) {
      currentDrive = targetDrive;
    } else if (delta > 0) {
      currentDrive += stepLimit;
    } else {
      currentDrive -= stepLimit;
    }

    driveValues[i] = currentDrive;
    if (
      motorStartBoostPending[i] &&
      abs(currentDrive) > motor.commandDeadband &&
      ((currentDrive > 0) == (targetDrive > 0))
    ) {
      activateMotorStartBoost(i);
    }
    applyMotorOutput(motors[i], currentDrive);
  }
}

// ---------------------------------------------------------------------------
// Operational Sensors
// ---------------------------------------------------------------------------

bool ads1115ReadRaw(uint8_t channel, int16_t& rawValue) {
  if (channel > 3) {
    return false;
  }

  // Single-shot, single-ended input, +/-4.096 V range, 860 samples/s.
  uint16_t config = 0x8000 | ((0x04 + channel) << 12) | 0x0200 | 0x0100 | 0x00E0 | 0x0003;
  Wire.beginTransmission(ADS1115_I2C_ADDRESS);
  Wire.write(0x01);
  Wire.write(static_cast<uint8_t>(config >> 8));
  Wire.write(static_cast<uint8_t>(config & 0xFF));
  if (Wire.endTransmission() != 0) {
    ads1115Available = false;
    return false;
  }

  delayMicroseconds(1400);
  Wire.beginTransmission(ADS1115_I2C_ADDRESS);
  Wire.write(0x00);
  if (Wire.endTransmission(false) != 0 || Wire.requestFrom(static_cast<int>(ADS1115_I2C_ADDRESS), 2, 1) != 2) {
    ads1115Available = false;
    return false;
  }
  rawValue = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  return true;
}

bool setupAds1115() {
  Wire.beginTransmission(ADS1115_I2C_ADDRESS);
  ads1115Available = Wire.endTransmission() == 0;
  return ads1115Available;
}

float readAds1115AverageVoltage(uint8_t channel, int sampleCount) {
  if (!ads1115Available && !setupAds1115()) {
    return NAN;
  }

  int64_t total = 0;
  int validSamples = 0;
  for (int i = 0; i < sampleCount; ++i) {
    int16_t raw = 0;
    if (ads1115ReadRaw(channel, raw)) {
      total += raw;
      ++validSamples;
    }
  }
  if (validSamples == 0) {
    return NAN;
  }
  return (total / static_cast<float>(validSamples)) * (ADS1115_FULL_SCALE_VOLTS / 32768.0f);
}

float readCurrentSensorVoltage() {
  float adcVoltage = readAds1115AverageVoltage(ADS1115_CURRENT_CHANNEL, CURRENT_SENSOR_SAMPLE_COUNT);
  return isnan(adcVoltage) ? NAN : adcVoltage / ACS712_INPUT_DIVIDER_RATIO;
}

float readBatteryAdcVoltage() {
  return readAds1115AverageVoltage(ADS1115_BATTERY_CHANNEL, BATTERY_VOLTAGE_SAMPLE_COUNT);
}

float readBatterySystemVoltage() {
  float sensorVoltage = readBatteryAdcVoltage();
  return isnan(sensorVoltage) ? 0.0f : sensorVoltage * (BATTERY_SENSOR_MAX_VOLTAGE / ADC_REFERENCE_VOLTAGE);
}

int estimateBatteryPercent(float batteryVoltage) {
  float normalized = (batteryVoltage - BATTERY_EMPTY_VOLTAGE) / (BATTERY_FULL_VOLTAGE - BATTERY_EMPTY_VOLTAGE);
  normalized = clampf(normalized, 0.0, 1.0);
  return round(normalized * 100.0);
}

void calibrateCurrentSensorZero() {
  float zeroVoltage = readAds1115AverageVoltage(ADS1115_CURRENT_CHANNEL, CURRENT_SENSOR_ZERO_CALIBRATION_SAMPLES);
  if (!isnan(zeroVoltage)) {
    currentSensorZeroVoltage = zeroVoltage / ACS712_INPUT_DIVIDER_RATIO;
  }
}

float readCurrentSensorAmps() {
  float sensorVoltage = readCurrentSensorVoltage();
  if (isnan(sensorVoltage)) {
    return 0.0f;
  }
  float currentAmps = -(sensorVoltage - currentSensorZeroVoltage) / ACS712_SENSITIVITY_VOLTS_PER_AMP;

  if (fabsf(currentAmps) < CURRENT_SENSOR_NOISE_FLOOR_AMPS) {
    currentAmps = 0.0;
  }

  return currentAmps;
}

void emitSensorDiagnostics() {
  pinMode(DS18B20_DATA_PIN, INPUT);

  float currentVoltage = readCurrentSensorVoltage();
  float batteryAdcVoltage = readBatteryAdcVoltage();
  float batteryVoltage = isnan(batteryAdcVoltage) ? 0.0f : batteryAdcVoltage * (BATTERY_SENSOR_MAX_VOLTAGE / ADC_REFERENCE_VOLTAGE);
  float currentAmps = -(currentVoltage - currentSensorZeroVoltage) / ACS712_SENSITIVITY_VOLTS_PER_AMP;
  emitLine("DIAG ADS1115 CURRENT_V " + String(currentVoltage, 3) + " CURRENT_A " + String(currentAmps, 3));
  emitLine("DIAG ADS1115 BATTERY_ADC_V " + String(batteryAdcVoltage, 3) + " BATTERY_V " + String(batteryVoltage, 3));
}

void printSdTimestamp(File& file, uint64_t timestampUs) {
  char text[24];
  snprintf(text, sizeof(text), "%llu", static_cast<unsigned long long>(timestampUs));
  file.print(text);
}

void printCsvFloat(File& file, float value, uint8_t decimals) {
  if (!isnan(value)) {
    file.print(value, decimals);
  }
}

void recordScienceSnapshot() {
  if (!sdRecordingAvailable) {
    return;
  }

  ImuReadings imu = {};
  bool imuValid = readMpu6050(imu);
  float currentVoltage = readCurrentSensorVoltage();
  float batteryAdcVoltage = readBatteryAdcVoltage();
  float tdsVoltage = readAds1115AverageVoltage(ADS1115_TDS_CHANNEL, 1);
  float turbidityAdcVoltage = readAds1115AverageVoltage(ADS1115_TURBIDITY_CHANNEL, 1);
  float turbidityProbeVoltage = isnan(turbidityAdcVoltage)
    ? NAN
    : turbidityAdcVoltage / TURBIDITY_INPUT_DIVIDER_RATIO;
  if (!isnan(currentVoltage)) {
    latestCurrentAmps = -(currentVoltage - currentSensorZeroVoltage) / ACS712_SENSITIVITY_VOLTS_PER_AMP;
    if (fabsf(latestCurrentAmps) < CURRENT_SENSOR_NOISE_FLOOR_AMPS) latestCurrentAmps = 0.0f;
  }
  if (!isnan(batteryAdcVoltage)) {
    latestBatteryVolts = batteryAdcVoltage * (BATTERY_SENSOR_MAX_VOLTAGE / ADC_REFERENCE_VOLTAGE);
  }

  printSdTimestamp(sdScienceFile, static_cast<uint64_t>(esp_timer_get_time()));
  sdScienceFile.print(',');
  sdScienceFile.print(imuValid ? 1 : 0);
  float imuValues[] = {
    imu.accelXG, imu.accelYG, imu.accelZG,
    imu.gyroXDps, imu.gyroYDps, imu.gyroZDps, imu.temperatureC,
  };
  for (float value : imuValues) {
    sdScienceFile.print(',');
    if (imuValid) sdScienceFile.print(value, 4);
  }
  sdScienceFile.print(',');
  if (ds18b20HasReading) sdScienceFile.print(ds18b20TemperatureC, 3);
  sdScienceFile.print(',');
  sdScienceFile.print(latestRangeValid ? 1 : 0);
  sdScienceFile.print(',');
  if (latestRangeValid) sdScienceFile.print(latestRangeDistanceMm);
  sdScienceFile.print(',');
  printCsvFloat(sdScienceFile, currentVoltage, 5);
  sdScienceFile.print(',');
  printCsvFloat(sdScienceFile, batteryAdcVoltage, 5);
  sdScienceFile.print(',');
  printCsvFloat(sdScienceFile, tdsVoltage, 5);
  sdScienceFile.print(',');
  printCsvFloat(sdScienceFile, turbidityProbeVoltage, 5);
  sdScienceFile.println();
  if (sdScienceFile.getWriteError()) {
    disableSdRecorder();
  }
}

void recordTelemetrySnapshot() {
  if (!sdRecordingAvailable) {
    return;
  }

  printSdTimestamp(sdTelemetryFile, static_cast<uint64_t>(esp_timer_get_time()));
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(gps.location.isValid() ? 1 : 0);
  sdTelemetryFile.print(',');
  if (gps.date.isValid()) sdTelemetryFile.print(gps.date.value());
  sdTelemetryFile.print(',');
  if (gps.time.isValid()) sdTelemetryFile.print(gps.time.value());
  sdTelemetryFile.print(',');
  if (gps.location.isValid()) sdTelemetryFile.print(gps.location.lat(), 7);
  sdTelemetryFile.print(',');
  if (gps.location.isValid()) sdTelemetryFile.print(gps.location.lng(), 7);
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(encodeControlMode(controlMode));
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(holdPositionEnabled ? 1 : 0);
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(motorsEnabled() ? 1 : 0);
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    sdTelemetryFile.print(',');
    sdTelemetryFile.print(driveValues[i]);
  }
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(latestBatteryVolts, 3);
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(latestCurrentAmps, 3);
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(audioFramesDropped);
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(sdRecordingAvailable ? 1 : 0);
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(sdMountFailures);
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(sdWriteFailures);
  uint64_t freeBytes = sdTotalBytes > sdUsedBytes ? sdTotalBytes - sdUsedBytes : 0;
  uint32_t storageMiB[] = {
    static_cast<uint32_t>(sdCardSizeBytes / (1024ULL * 1024ULL)),
    static_cast<uint32_t>(sdTotalBytes / (1024ULL * 1024ULL)),
    static_cast<uint32_t>(sdUsedBytes / (1024ULL * 1024ULL)),
    static_cast<uint32_t>(freeBytes / (1024ULL * 1024ULL)),
  };
  for (uint32_t value : storageMiB) {
    sdTelemetryFile.print(',');
    sdTelemetryFile.print(value);
  }
  sdTelemetryFile.print(',');
  printSdTimestamp(sdTelemetryFile, static_cast<uint64_t>(sdAudioFramesWritten) * 2ULL * sizeof(int16_t));
  sdTelemetryFile.print(',');
  sdTelemetryFile.print(sdAudioWriteBufferUsed);
  sdTelemetryFile.print(',');
  sdTelemetryFile.println(audioFrameQueue == nullptr ? 0 : uxQueueMessagesWaiting(audioFrameQueue));
  if (sdTelemetryFile.getWriteError()) {
    disableSdRecorder();
  }
}

void updateSdCardTelemetry() {
  if (!sdCardMounted) {
    return;
  }
  sdCardSizeBytes = SD.cardSize();
  sdTotalBytes = SD.totalBytes();
  sdUsedBytes = SD.usedBytes();
}

void serviceSdRecorder(uint32_t nowMs) {
  if (!sdRecordingAvailable) {
    if (nowMs - lastSdCardTelemetryMs >= SD_CARD_TELEMETRY_INTERVAL_MS) {
      lastSdCardTelemetryMs = nowMs;
      updateSdCardTelemetry();
      if (controlClientConnected()) {
        emitLine(buildSdStatusLine());
      }
    }
    return;
  }
  if (scienceExperimentState != 1) return;
  uint32_t sensorLogIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : SD_SCIENCE_INTERVAL_MS;
  uint32_t telemetryLogIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : SD_TELEMETRY_INTERVAL_MS;
  if (nowMs - lastSdScienceMs >= sensorLogIntervalMs) {
    lastSdScienceMs = nowMs;
    recordScienceSnapshot();
  }
  if (nowMs - lastSdTelemetryMs >= telemetryLogIntervalMs) {
    lastSdTelemetryMs = nowMs;
    recordTelemetrySnapshot();
  }
  if (nowMs - lastSdCardTelemetryMs >= SD_CARD_TELEMETRY_INTERVAL_MS) {
    lastSdCardTelemetryMs = nowMs;
    updateSdCardTelemetry();
  }
  if (nowMs - lastSdFlushMs >= SD_FLUSH_INTERVAL_MS) {
    lastSdFlushMs = nowMs;
    if (!flushAudioWriteBuffer()) {
      return;
    }
    updateWavHeader();
    sdAudioFile.flush();
    if (sdAudioIndexFile) sdAudioIndexFile.flush();
    sdScienceFile.flush();
    sdTelemetryFile.flush();
    if (
      sdAudioFile.getWriteError() || (sdAudioIndexFile && sdAudioIndexFile.getWriteError()) ||
      sdScienceFile.getWriteError() || sdTelemetryFile.getWriteError()
    ) {
      disableSdRecorder();
    }
  }
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
  config.sample_rate = audioSampleRateHz;
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

void audioCaptureTask(void* parameter) {
  int32_t rawSamples[AUDIO_FRAME_SAMPLES * 2];
  AudioFrame frame = {};

  while (true) {
    if (serviceMode) {
      vTaskDelay(pdMS_TO_TICKS(20));
      continue;
    }
    size_t bytesRead = 0;
    esp_err_t result = i2s_read(
      AUDIO_I2S_PORT,
      rawSamples,
      sizeof(rawSamples),
      &bytesRead,
      portMAX_DELAY
    );
    if (result != ESP_OK || bytesRead == 0) {
      continue;
    }

    size_t sampleCount = bytesRead / sizeof(int32_t);
    frame.frameCount = static_cast<uint16_t>(sampleCount / 2);
    if (frame.frameCount == 0 || frame.frameCount > AUDIO_FRAME_SAMPLES) {
      continue;
    }
    // Timestamp the first sample approximately; the I2S read completes after
    // the captured buffer, so subtract its duration from the monotonic clock.
    uint64_t durationUs = (static_cast<uint64_t>(frame.frameCount) * 1000000ULL) / audioSampleRateHz;
    frame.timestampUs = static_cast<uint64_t>(esp_timer_get_time()) - durationUs;
    for (size_t i = 0; i < frame.frameCount * 2; ++i) {
      int32_t sample = rawSamples[i] >> AUDIO_PCM_SHIFT;
      sample = static_cast<int32_t>(sample * AUDIO_INPUT_GAIN);
      frame.samples[i] = static_cast<int16_t>(constrain(sample, -32768, 32767));
    }

    if (xQueueSend(audioFrameQueue, &frame, 0) != pdTRUE) {
      ++audioFramesDropped;
    }
  }
}

bool startAudioCaptureTask() {
  if (!audioInputAvailable) {
    return false;
  }
  audioFrameQueue = xQueueCreate(AUDIO_QUEUE_FRAME_COUNT, sizeof(AudioFrame));
  if (audioFrameQueue == nullptr) {
    return false;
  }
  return xTaskCreatePinnedToCore(
    audioCaptureTask,
    "audio-capture",
    6144,
    nullptr,
    3,
    &audioCaptureTaskHandle,
    0
  ) == pdPASS;
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

void recordAudioFrame(const AudioFrame& frame) {
  if (!sdRecordingAvailable || !audioRecordingEnabled || frame.frameCount == 0) {
    return;
  }

  size_t byteCount = frame.frameCount * 2 * sizeof(int16_t);
  if (byteCount > sizeof(sdAudioWriteBuffer)) {
    ++sdWriteFailures;
    return;
  }
  if (sdAudioWriteBufferUsed + byteCount > sizeof(sdAudioWriteBuffer) && !flushAudioWriteBuffer()) {
    return;
  }
  if (sdAudioWriteBufferUsed == 0) {
    sdAudioBufferTimestampUs = frame.timestampUs;
    sdAudioBufferFirstFrame = sdAudioFramesWritten;
  }

  AudioIndexRecord indexRecord = {
    frame.timestampUs,
    sdAudioFramesWritten,
    frame.frameCount,
    0,
  };
  if (sdAudioIndexFile.write(reinterpret_cast<const uint8_t*>(&indexRecord), sizeof(indexRecord)) != sizeof(indexRecord)) {
    disableSdRecorder();
    return;
  }
  memcpy(sdAudioWriteBuffer + sdAudioWriteBufferUsed, frame.samples, byteCount);
  sdAudioWriteBufferUsed += byteCount;
  sdAudioFramesWritten += frame.frameCount;
  if (sdAudioWriteBufferUsed == sizeof(sdAudioWriteBuffer)) {
    flushAudioWriteBuffer();
  }
}

void processAudioFrames() {
  if (audioFrameQueue == nullptr) {
    return;
  }
  AudioFrame frame;
  for (uint8_t handled = 0; handled < 32; ++handled) {
    if (xQueueReceive(audioFrameQueue, &frame, 0) != pdTRUE) {
      break;
    }
    uint8_t channelSelection = audioChannelSelection;
    if (channelSelection == 1) {
      for (size_t i = 1; i < frame.frameCount * 2; i += 2) frame.samples[i] = 0;
    } else if (channelSelection == 2) {
      for (size_t i = 0; i < frame.frameCount * 2; i += 2) frame.samples[i] = 0;
    }
    recordAudioFrame(frame);
    if (audioStreamEnabled) sendUdpAudioPacket(frame.samples, frame.frameCount);
  }
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
    setMotorDrive(i, appliedDrive);
  }
}

void applyDirectMotorDriveAll(int driveValue) {
  int clampedDriveValue = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  controlMode = CONTROL_MODE_MANUAL;
  holdPositionEnabled = false;

  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, clampedDriveValue);
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

  // Only explicit control traffic and the control-station heartbeat refresh
  // the propulsion watchdog. Telemetry requests cannot keep a stale motor
  // command alive.
  if (
    command == "PING" || command == "STOP" || command == "BANANA" ||
    command.startsWith("CTRL ")
  ) {
    markControlHeartbeat();
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

  if (command == "CTRL SERVICE START") {
    if (scienceExperimentState != 0) { emitLine("ERR STOP AND SAVE MISSION FIRST"); return; }
    stopAllMotors();
    controlMode = CONTROL_MODE_IDLE;
    holdPositionEnabled = false;
    audioStreamEnabled = false;
    imuStreamEnabled = false;
    serviceMode = true;
    if (audioFrameQueue) xQueueReset(audioFrameQueue);
    emitLine("ACK CTRL SERVICE START");
    sendStatusSnapshot();
    return;
  }
  if (command == "CTRL SERVICE STOP") {
    serviceMode = false;
    emitLine("ACK CTRL SERVICE STOP");
    sendStatusSnapshot();
    return;
  }
  if (serviceMode && command.startsWith("CTRL ") && command != "CTRL STOP") {
    emitLine("ERR SERVICE MODE ACTIVE");
    return;
  }

  if (command == "CTRL STREAM AUDIO START" || command == "CTRL STREAM AUDIO STOP") {
    audioStreamEnabled = command.endsWith("START");
    emitLine(String("ACK CTRL STREAM AUDIO ") + (audioStreamEnabled ? "START" : "STOP"));
    sendStatusSnapshot();
    return;
  }
  if (command == "CTRL STREAM IMU START" || command == "CTRL STREAM IMU STOP") {
    imuStreamEnabled = command.endsWith("START");
    emitLine(String("ACK CTRL STREAM IMU ") + (imuStreamEnabled ? "START" : "STOP"));
    sendStatusSnapshot();
    return;
  }
  if (command.startsWith("CTRL SCIENCE START")) {
    if (scienceExperimentState != 0 || !sdCardMounted || sdTransferActive) {
      emitLine("ERR SCIENCE START UNAVAILABLE");
      return;
    }
    String startUtc = command.substring(18);
    startUtc.trim();
    if (startUtc.length() == 0) startUtc = "BOOT-" + String(millis());
    for (size_t i = 0; i < startUtc.length(); ++i) {
      char c = startUtc.charAt(i);
      if (!((c >= '0' && c <= '9') || c == 'T' || c == 'Z' || c == '-')) {
        emitLine("ERR BAD SCIENCE TIMESTAMP");
        return;
      }
    }
    if (!startSdRecorder(startUtc)) {
      emitLine("ERR SCIENCE SESSION CREATE FAILED");
      return;
    }
    emitLine("ACK CTRL SCIENCE START " + sdSessionDirectory);
    sendStatusSnapshot();
    return;
  }
  if (command.startsWith("CTRL SCIENCE STOP")) {
    if (scienceExperimentState != 1) { emitLine("ERR SCIENCE NOT RECORDING"); return; }
    String stopUtc = command.substring(17);
    stopUtc.trim();
    audioRecordingEnabled = false;
    if (!flushAudioWriteBuffer()) { emitLine("ERR SCIENCE SD WRITE FAILED"); return; }
    updateWavHeader();
    sdAudioFile.flush();
    sdAudioIndexFile.flush();
    sdScienceFile.flush();
    sdTelemetryFile.flush();
    File manifest = SD.open((sdSessionDirectory + "/manifest.txt").c_str(), FILE_APPEND);
    if (manifest) { manifest.println("stopped_utc=" + stopUtc); manifest.close(); }
    scienceExperimentState = 2;
    emitLine("ACK CTRL SCIENCE STOP " + sdSessionDirectory);
    sendStatusSnapshot();
    return;
  }
  if (command.startsWith("CTRL SCIENCE SAVE")) {
    if (sdTransferActive || sdAuxFilesPaused) { emitLine("ERR SD TRANSFER BUSY"); return; }
    String saveUtc = command.substring(17);
    saveUtc.trim();
    if (!saveSdRecorder(saveUtc)) { emitLine("ERR SCIENCE SAVE FAILED"); return; }
    emitLine("ACK CTRL SCIENCE SAVE " + sdSessionDirectory);
    sendStatusSnapshot();
    return;
  }

  if (command.startsWith("CTRL AUDIO CHANNEL ")) {
    String selection = command.substring(19);
    selection.trim();
    if (selection == "BOTH") audioChannelSelection = 0;
    else if (selection == "LEFT") audioChannelSelection = 1;
    else if (selection == "RIGHT") audioChannelSelection = 2;
    else {
      emitLine("ERR BAD CTRL AUDIO CHANNEL");
      return;
    }
    emitLine("ACK CTRL AUDIO CHANNEL " + selection);
    sendStatusSnapshot();
    return;
  }

  if (command.startsWith("CTRL AUDIO QUALITY ")) {
    if (audioRecordingEnabled || sdAudioFramesWritten > 0) {
      emitLine("ERR AUDIO QUALITY LOCKED; NEW SESSION REQUIRED");
      return;
    }
    String quality = command.substring(19);
    quality.trim();
    uint32_t requestedRate = quality == "LOW" ? 8000UL
      : (quality == "DEFAULT" ? AUDIO_DEFAULT_SAMPLE_RATE : (quality == "MAX" ? 48000UL : 0UL));
    if (requestedRate == 0) {
      emitLine("ERR BAD CTRL AUDIO QUALITY");
      return;
    }
    if (requestedRate != audioSampleRateHz) {
      esp_err_t result = i2s_set_clk(
        AUDIO_I2S_PORT,
        requestedRate,
        I2S_BITS_PER_SAMPLE_32BIT,
        I2S_CHANNEL_STEREO
      );
      if (result != ESP_OK) {
        emitLine("ERR AUDIO QUALITY APPLY FAILED");
        return;
      }
      audioSampleRateHz = requestedRate;
      i2s_zero_dma_buffer(AUDIO_I2S_PORT);
      if (audioFrameQueue != nullptr) xQueueReset(audioFrameQueue);
      if (sdRecordingAvailable) updateWavHeader();
    }
    emitLine("ACK CTRL AUDIO QUALITY " + quality + " " + String(audioSampleRateHz));
    sendStatusSnapshot();
    return;
  }

  if (command == "CTRL AUDIO RECORD START") {
    if (!sdRecordingAvailable || scienceExperimentState != 1) {
      emitLine("ERR START SCIENCE EXPERIMENT FIRST");
      return;
    }
    audioRecordingEnabled = true;
    emitLine("ACK CTRL AUDIO RECORD START");
    sendStatusSnapshot();
    return;
  }

  if (command == "CTRL AUDIO RECORD STOP") {
    audioRecordingEnabled = false;
    if (sdRecordingAvailable) {
      if (!flushAudioWriteBuffer()) {
        emitLine("ERR AUDIO SD WRITE FAILED");
        return;
      }
      updateWavHeader();
      sdAudioFile.flush();
      sdAudioIndexFile.flush();
    }
    emitLine("ACK CTRL AUDIO RECORD STOP");
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

  if (command.startsWith("CTRL MOTOR PERCENT ")) {
    String payload = command.substring(19);
    payload.trim();

    char motorName[20];
    int powerPercent = 0;
    if (sscanf(payload.c_str(), "%19s %d", motorName, &powerPercent) != 2) {
      emitLine("ERR BAD CTRL MOTOR PERCENT");
      return;
    }
    if (powerPercent < -100 || powerPercent > 100) {
      emitLine("ERR RANGE CTRL MOTOR PERCENT");
      return;
    }

    String motorNameText = String(motorName);
    int motorIndex = motorIndexFromName(motorNameText);
    if (motorIndex < 0) {
      emitLine("ERR BAD CTRL MOTOR NAME");
      return;
    }

    int drive = static_cast<int>(round(powerPercent * MOTOR_OUTPUT_LIMIT / 100.0f));
    if (motorIndex == MOTOR_COUNT) {
      if (drive == 0) {
        stopAllMotors();
        controlMode = CONTROL_MODE_IDLE;
        holdPositionEnabled = false;
      } else {
        applyDirectMotorDriveAll(drive);
      }
    } else {
      applyDirectMotorDrive(motorIndex, drive);
    }
    emitLine(
      "ACK CTRL MOTOR PERCENT " + motorNameText + " " + String(powerPercent) +
      " DRIVE " + String(drive)
    );
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

  int lateral = 0;
  int thrust = 0;
  int yaw = 0;
  int motionMatched = sscanf(command.c_str(), "CTRL MOTION %d %d %d", &lateral, &thrust, &yaw);
  if (motionMatched == 3) {
    if (
      lateral < -100 || lateral > 100 ||
      thrust < -100 || thrust > 100 ||
      yaw < -100 || yaw > 100
    ) {
      emitLine("ERR RANGE CTRL MOTION");
      return;
    }

    applyMotionCommand(lateral, thrust, yaw);
    controlMode = CONTROL_MODE_MANUAL;
    holdPositionEnabled = false;
    emitLine(
      "ACK CTRL MOTION " + String(lateral) + " " + String(thrust) + " " + String(yaw)
    );
    return;
  }

  int vectorMatched = sscanf(command.c_str(), "CTRL VECTOR %d %d", &lateral, &thrust);
  if (vectorMatched == 2) {
    if (lateral < -100 || lateral > 100 || thrust < -100 || thrust > 100) {
      emitLine("ERR RANGE CTRL VECTOR");
      return;
    }

    applyVectorCommand(lateral, thrust);
    controlMode = CONTROL_MODE_MANUAL;
    holdPositionEnabled = false;
    emitLine("ACK CTRL VECTOR " + String(lateral) + " " + String(thrust));
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
    // Drive both bridge inputs low before handing them to LEDC so the motor
    // driver never sees a floating or partially configured startup state.
    pinMode(motors[i].input1Pin, OUTPUT);
    digitalWrite(motors[i].input1Pin, LOW);
    pinMode(motors[i].input2Pin, OUTPUT);
    digitalWrite(motors[i].input2Pin, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(motors[i].input1Pin, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttach(motors[i].input2Pin, PWM_FREQ_HZ, PWM_RES_BITS);
#else
    ledcSetup(motors[i].input1PwmChannel, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(motors[i].input1Pin, motors[i].input1PwmChannel);
    ledcSetup(motors[i].input2PwmChannel, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(motors[i].input2Pin, motors[i].input2PwmChannel);
#endif

    writeMotorPwm(motors[i].input1Pin, motors[i].input1PwmChannel, 0);
    writeMotorPwm(motors[i].input2Pin, motors[i].input2PwmChannel, 0);
  }
}

void setup() {
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  lastLedToggleMs = millis();
  setupMotors();
  stopAllMotors();
  ads1115Available = setupAds1115();
  sdCardMounted = mountSdCard();
  setupWifiControl();
  delay(250);
  calibrateCurrentSensorZero();
  setupDs18b20();
  setupJsnSr04t();
  mpu6050Available = setupMpu6050();
  audioInputAvailable = setupAudioInput();
  if (audioInputAvailable && !startAudioCaptureTask()) {
    audioInputAvailable = false;
  }

  emitLine(FIRMWARE_BANNER);
  emitLine("ESP32 buoy firmware ready");
  emitLine("SD card: " + String(sdCardMounted ? "ready" : "unavailable"));
  emitLine("ADS1115: " + String(ads1115Available ? "ready" : "unavailable"));
  emitLine("MPU6050: " + String(mpu6050Available ? "ready" : "unavailable"));
  emitLine("INMP441 audio: " + String(audioInputAvailable ? "ready" : "unavailable"));
  updateSdCardTelemetry();
  emitSensorDiagnostics();
}

void loop() {
  if (!serviceMode) processAudioFrames();
  processTcpControl();
  processSdFileServer();
  updateMotorCommandWatchdog();
  if (serviceMode) {
    updateStatusLed();
    delay(1);
    return;
  }
  processGpsSerial();
  updateStatusLed();
  updateMotorRamps();

  uint32_t nowMs = millis();
  updateDs18b20Sensor(nowMs);
  uint32_t sensorStreamIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : IMU_STREAM_INTERVAL_MS;
  uint32_t rangeIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : RANGE_STREAM_INTERVAL_MS;
  uint32_t waterTemperatureIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : WATER_TEMP_STREAM_INTERVAL_MS;
  uint32_t powerIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : POWER_STREAM_INTERVAL_MS;
  uint32_t diagnosticsIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : 5000UL;
  if (imuStreamEnabled && controlClientConnected() && (nowMs - lastImuStreamMs >= sensorStreamIntervalMs)) {
    lastImuStreamMs = nowMs;
    sendUdpImuTelemetry();
  }
  if (!sdTransferActive && nowMs - lastRangeStreamMs >= rangeIntervalMs) {
    lastRangeStreamMs = nowMs;
    latestRangeValid = readJsnSr04tDistanceMm(latestRangeDistanceMm);
    if (controlClientConnected()) {
      sendUdpRangeTelemetry();
    }
  }
  if (controlClientConnected() && (nowMs - lastWaterTempStreamMs >= waterTemperatureIntervalMs)) {
    lastWaterTempStreamMs = nowMs;
    emitWaterTemperatureTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastPowerStreamMs >= powerIntervalMs)) {
    lastPowerStreamMs = nowMs;
    sendUdpPowerTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastStateStreamMs >= STATE_STREAM_INTERVAL_MS)) {
    lastStateStreamMs = nowMs;
    sendUdpStateTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastMotorOutputStreamMs >= MOTOR_OUTPUT_STREAM_INTERVAL_MS)) {
    lastMotorOutputStreamMs = nowMs;
    sendMotorOutputTelemetry();
  }
  if (controlClientConnected() && (nowMs - lastGpsStreamMs >= GPS_STREAM_INTERVAL_MS)) {
    lastGpsStreamMs = nowMs;
    sendUdpGpsTelemetry();
  }
  serviceSdRecorder(nowMs);
  processAudioFrames();

  if (nowMs - lastSerialSensorTestMs >= diagnosticsIntervalMs) {
    lastSerialSensorTestMs = nowMs;
    emitSensorDiagnostics();
  }
}


