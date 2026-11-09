#include <TinyGPSPlus.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <Preferences.h>
#include <math.h>
#include <driver/i2s.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <lwip/sockets.h>
#include <stdlib.h>
#include <string.h>
#include "elrs_link.h"
#include "firmware_version.h"
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
const uint8_t QMC5883_I2C_ADDRESS = 0x0D;
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
const uint32_t SD_STANDALONE_TELEMETRY_INTERVAL_MS = 1000;
const uint32_t SD_SCIENCE_INTERVAL_MS = 100;
const uint32_t SD_TELEMETRY_INTERVAL_MS = 100;
const uint32_t AUDIO_PRIORITY_SENSOR_INTERVAL_MS = 30000;

// UART roles
// UART0: USB programming/debug (Serial)
// UART1: GPS
// UART2: SkyGuy Nano ELRS receiver, TX -> GPIO34 and RX <- GPIO25
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = -1;  // Receive-only; GPIO17 is the SD clock.

const long GPS_BAUDRATE = 115200;
const char* const WIFI_STA_SSID = "Bouy";
const char* const WIFI_STA_PASSWORD = "SuperMonkey";
const uint16_t WIFI_TCP_PORT = 5000;
const uint16_t WIFI_UDP_PORT = 5001;
const uint16_t WIFI_AUDIO_UDP_PORT = 5002;
const uint16_t WIFI_SD_FILE_PORT = 5003;

const float ADC_REFERENCE_VOLTAGE = 3.3;
const float ADC_COUNTS = 4095.0;
const float ACS712_SENSITIVITY_VOLTS_PER_AMP = 0.066;  // 30A module
const float BATTERY_SENSOR_MAX_VOLTAGE = 16.72f;  // 12.39 V pack at 2.445 V on ADS1115 A1
const float BATTERY_FULL_VOLTAGE = 12.6;
const float BATTERY_EMPTY_VOLTAGE = 9.6;
const float BATTERY_MIN_VALID_PACK_VOLTAGE = 7.0f;
const float BATTERY_MAX_VALID_PACK_VOLTAGE = 15.0f;
const uint32_t BATTERY_RESTING_PERCENT_MAX_AGE_MS = 120000;
const uint32_t MISSION_LOW_BATTERY_SUSTAIN_MS = 10000;
const uint8_t MISSION_LOW_BATTERY_HYSTERESIS_PERCENT = 5;
const int CURRENT_SENSOR_SAMPLE_COUNT = 8;
const int BATTERY_VOLTAGE_SAMPLE_COUNT = 4;
const int CURRENT_SENSOR_ZERO_CALIBRATION_SAMPLES = 64;
const float CURRENT_SENSOR_NOISE_FLOOR_AMPS = 0.15;
const float CURRENT_SENSOR_STARTUP_LOAD_AMPS = 0.21f;  // Measured at battery feed with motors off
const float MOTOR_STALL_OVERCURRENT_AMPS = 12.0f;
const uint32_t MOTOR_STALL_OVERCURRENT_MS = 5000;
const float MOTOR_STALL_GPS_MOVEMENT_METERS = 2.0f;
const uint32_t MOTOR_STALL_SAMPLE_INTERVAL_MS = 250;

const int MOTOR_OUTPUT_LIMIT = 255;
// Below this command magnitude, the calibrated sustain PWM is already near its
// minimum. Pulse the whole mix on a common clock so relative motor on-time is
// proportional instead of flattening every small command to the same PWM.
const int LOW_SPEED_MIX_THRESHOLD = 64;
const uint32_t LOW_SPEED_MIX_PERIOD_MS = 800;
const int VECTOR_SCALE = 100;
const float MOTOR_OUTPUT_BOOST = 2.0f;
const bool MOTOR_DIRECTION_INVERTED = true;
const uint32_t MOTOR_RAMP_INTERVAL_MS = 20;
const int MOTOR_RAMP_UP_PER_SECOND = 120;
const int MOTOR_RAMP_DOWN_PER_SECOND = 200;
const uint32_t MOTOR_COMMAND_WATCHDOG_MS = 1000;
const int ELRS_MOTOR_POWER_LIMIT_PERCENT = 100;
const uint32_t MANUAL_ASSIST_UPDATE_INTERVAL_MS = 100;
const float MANUAL_ASSIST_HEADING_DEADBAND_DEG = 4.0f;
const float MANUAL_ASSIST_YAW_GAIN_PERCENT_PER_DEG = 0.55f;
const int MANUAL_ASSIST_MAX_YAW_PERCENT = 25;
const uint32_t ELRS_TELEMETRY_INTERVAL_MS = 1000;
const uint32_t ELRS_BATTERY_INTERVAL_MS = 5000;
const uint32_t ELRS_GPS_INTERVAL_MS = 5000;
const uint32_t ELRS_SCIENCE_INTERVAL_MS = 1000;
const uint32_t ELRS_NAVIGATION_INTERVAL_MS = 5000;
const uint32_t ELRS_ATTITUDE_INTERVAL_MS = 1000;
const uint32_t ELRS_MOTION_SAMPLE_INTERVAL_MS = 100;
const uint32_t ELRS_MOTION_WINDOW_MS = 5000;
const uint32_t ELRS_CALIBRATION_SAMPLE_INTERVAL_MS = 50;
const uint32_t ELRS_CALIBRATION_PULSE_DURATION_DEFAULT_MS = 10000;
const uint32_t ELRS_CALIBRATION_PULSE_DURATION_LONG_MS = 20000;
// EdgeTX extended channel limits can send 881..2119 us on CH11-CH16.
// Accept those endpoints for calibration metadata while still rejecting
// missing/ignored (0) channels and values outside the RC override range.
const uint16_t ELRS_CALIBRATION_CHANNEL_MIN_US = 881;
const uint16_t ELRS_CALIBRATION_CHANNEL_MAX_US = 2119;

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
const uint32_t WATER_CHEMISTRY_STREAM_INTERVAL_MS = 2000;
const uint32_t POWER_STREAM_INTERVAL_MS = 500;
const uint32_t STATE_STREAM_INTERVAL_MS = 500;
const uint32_t MOTOR_OUTPUT_STREAM_INTERVAL_MS = 100;
const uint32_t ELRS_CHANNEL_STREAM_INTERVAL_MS = 100;
const uint32_t GPS_STREAM_INTERVAL_MS = 500;
const uint32_t MAGNETOMETER_STREAM_INTERVAL_MS = 500;
// The QMC5883 updates at 50 Hz while mission navigation updates every 200 ms.
// Keep the last valid heading across brief not-ready samples, but stop if it
// goes stale or the sensor reports an I2C failure.
const uint32_t MISSION_HEADING_MAX_AGE_MS = 500;
const double MISSION_START_RADIUS_M = 30.0;
const size_t TCP_COMMAND_BUFFER_LIMIT = 600;
const i2s_port_t AUDIO_I2S_PORT = I2S_NUM_0;
const uint32_t AUDIO_DEFAULT_SAMPLE_RATE = 16000;
const size_t AUDIO_FRAME_SAMPLES = 256;
// INMP441 samples are 24-bit values in 32-bit I2S slots; retain the upper
// 16 bits when converting to signed 16-bit WAV samples.
const int AUDIO_PCM_SHIFT = 16;
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

struct MagnetometerReadings {
  int16_t x;
  int16_t y;
  int16_t z;
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
  PACKET_TYPE_ELRS_CHANNELS = 6,
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

struct __attribute__((packed)) ElrsChannelsPacketPayload {
  uint16_t channels[16];
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
  {REAR_IN1_PIN, REAR_IN2_PIN, REAR_IN1_PWM_CHANNEL, REAR_IN2_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, false, 0, 110, 110, 0, 0, 110, 110, 255, 255, 2.00f, 2.00f, 1.00f, 1.00f, 0, 0, 75, 75},
  {FRONT_LEFT_IN1_PIN, FRONT_LEFT_IN2_PIN, FRONT_LEFT_IN1_PWM_CHANNEL, FRONT_LEFT_IN2_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, false, 0, 110, 110, 0, 0, 110, 110, 255, 255, 2.00f, 2.00f, 1.00f, 1.00f, 0, 0, 75, 75},
  {FRONT_RIGHT_IN1_PIN, FRONT_RIGHT_IN2_PIN, FRONT_RIGHT_IN1_PWM_CHANNEL, FRONT_RIGHT_IN2_PWM_CHANNEL, 1.00f, 1.00f, 0.00f, false, 0, 110, 110, 0, 0, 110, 110, 255, 255, 2.00f, 2.00f, 1.00f, 1.00f, 0, 0, 75, 75},
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
// In the single-motor water trial, rear+ and front-left+ turned the hull the
// same way, while front-right+ turned it the opposite way. Manual strafe must
// counter that torque while retaining a net lateral force.
const float tx12_manual_lateral_gains[3] = {-1.0f, 0.5f, -0.5f};
// Right strafe was still producing a clockwise yaw bias in the water trace.
// Move some reverse demand off the rear motor and into the front-left motor;
// leave the opposite (left strafe) mix unchanged for this trial.
const float tx12_manual_right_lateral_gains[3] = {-0.8f, 0.8f, -0.5f};
// Empirical mixer points are commanded motor drive percentages at each
// requested axis magnitude. Axis order: forward, reverse, right, left, CW, CCW.
const uint8_t MIXER_CAL_POWER_STEPS = 10;  // 10..100 percent, in 10 percent steps.
const uint8_t MIXER_CAL_AXIS_COUNT = 6;
int8_t mixerCalValues[MIXER_CAL_POWER_STEPS][MIXER_CAL_AXIS_COUNT][MOTOR_COUNT] = {};
bool mixerCalPointSet[MIXER_CAL_POWER_STEPS][MIXER_CAL_AXIS_COUNT] = {};
bool mixerCalProfileLoaded = false;
bool mixerCalLoadedFromBackup = false;
uint32_t mixerCalProfileCrc = 0;
// Water-trial starting estimate: a reverse-running propeller produces about
// half the useful thrust of the same forward command.
const float TX12_MANUAL_REVERSE_COMMAND_GAIN = 2.0f;

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------

HardwareSerial gpsSerial(1);   // UART1
TinyGPSPlus gps;
HardwareSerial elrsSerial(2);  // UART2
ElrsLink elrs(elrsSerial);

enum ControlSource { CONTROL_SOURCE_NONE, CONTROL_SOURCE_WIFI, CONTROL_SOURCE_ELRS };
ControlSource controlSource = CONTROL_SOURCE_NONE;
enum TelemetryMode { TELEMETRY_MODE_WIFI, TELEMETRY_MODE_MAVLINK };
TelemetryMode telemetryMode = TELEMETRY_MODE_WIFI;
bool elrsArmReady = false;
bool elrsArmed = false;
bool elrsArmBlockedByWifiLogged = false;
bool elrsArmInterlockBlockLogged = false;
bool magnetometerAvailable = false;
bool magnetometerCalibrationLoaded = false;
float magnetometerOffsetX = 0.0f;
float magnetometerOffsetY = 0.0f;
float magnetometerScaleX = 1.0f;
float magnetometerScaleY = 1.0f;
bool missionHeadingCacheValid = false;
float missionHeadingCacheDegrees = 0.0f;
uint32_t missionHeadingCacheUpdatedMs = 0;
MagnetometerReadings lastMagnetometerReadings = {};
bool lastMagnetometerSampleValid = false;
uint32_t lastMagnetometerSampleMs = 0;
uint8_t lastMagnetometerStatus = 0;
uint32_t magnetometerSamplesOk = 0;
uint32_t magnetometerNotReady = 0;
uint32_t magnetometerOverflows = 0;
uint32_t magnetometerI2cErrors = 0;
bool missionModeRequested = false;
bool manualAssistRequested = false;
bool manualAssistTargetValid = false;
bool manualAssistYawing = false;
bool manualAssistHeadingAvailable = false;
float manualAssistTargetHeadingDegrees = 0.0f;
float manualAssistLastYawCorrection = 0.0f;
uint32_t lastManualAssistUpdateMs = 0;
bool elrsScienceSwitchPositionKnown = false;
int8_t elrsScienceSwitchPosition = 0;
bool elrsScienceManualModeKnown = false;
bool elrsScienceManualMode = false;
bool manualLinkLossStop = false;
uint32_t elrsFailsafeCount = 0;
String lastElrsFailsafeStatus = "NONE";
uint32_t lastElrsAppliedRcMs = 0;
uint32_t lastElrsTelemetryMs = 0;
uint32_t lastElrsBatteryMs = 0;
uint32_t lastElrsGpsMs = 0;
uint32_t lastWifiHomeStatusMs = 0;
uint8_t lastElrsHomeKind = 255;
double lastElrsHomeLatitude = NAN;
double lastElrsHomeLongitude = NAN;
uint32_t lastElrsScienceMs = 0;
uint32_t lastElrsNavigationMs = 0;
uint32_t lastElrsAttitudeMs = 0;
uint32_t lastElrsMotionSampleMs = 0;
uint32_t elrsMotionWindowStartMs = 0;
float elrsDynamicAccelSquaredSum = 0.0f;
float elrsAngularRateSquaredSum = 0.0f;
uint16_t elrsMotionSampleCount = 0;
uint8_t elrsScienceField = 0;

double gpsLatitude = 0.0;
double gpsLongitude = 0.0;

int driveValues[] = {0, 0, 0};
int targetDriveValues[] = {0, 0, 0};
int appliedMotorPwm[] = {0, 0, 0};
bool motorStartBoostPending[] = {false, false, false};
uint32_t motorStartBoostUntilMs[] = {0, 0, 0};
uint32_t lowSpeedMixEpochMs = 0;
ControlMode controlMode = CONTROL_MODE_IDLE;
bool holdPositionEnabled = false;
bool targetLocationSet = false;
float targetLatitude = 0.0;
float targetLongitude = 0.0;
const uint8_t MAX_MISSION_WAYPOINTS = 64;
const uint8_t MAX_MISSION_BOUNDARY_POINTS = 64;
struct MissionCoordinate { float latitude; float longitude; };
struct MissionWaypoint { float latitude; float longitude; uint16_t measurementMask; bool collectData; uint16_t holdSeconds; uint8_t speedPercent; };
struct MissionMetadata {
  bool hasHome = false;
  bool hasEnd = false;
  MissionCoordinate home = {};
  MissionCoordinate end = {};
  uint8_t boundaryCount = 0;
  MissionCoordinate boundary[MAX_MISSION_BOUNDARY_POINTS] = {};
  uint8_t linkLossAction = 1; // 0=stop, 1=return home
  uint8_t boundaryBreachAction = 1;
  uint8_t lowBatteryAction = 1;
  uint8_t batteryThresholdPercent = 20;
};
MissionWaypoint missionWaypoints[MAX_MISSION_WAYPOINTS] = {};
MissionWaypoint stagedMissionWaypoints[MAX_MISSION_WAYPOINTS] = {};
MissionMetadata missionMetadata;
MissionCoordinate missionPlannedHomeBeforeRun = {};
bool missionHomeCapturedForRun = false;
MissionMetadata stagedMissionMetadata;
uint8_t missionWaypointCount = 0;
uint8_t stagedMissionWaypointCount = 0;
uint64_t stagedMissionWaypointMask = 0;
uint64_t stagedMissionBoundaryMask = 0;
uint8_t stagedMissionBoundaryCount = 0;
bool stagedMissionHasHome = false;
bool stagedMissionHasEnd = false;
bool stagedMissionHomeReceived = false;
bool stagedMissionEndReceived = false;
bool missionUploadInProgress = false;
char missionId[37] = "";
char stagedMissionId[37] = "";
char runningMissionId[37] = "";
bool missionExecutionActive = false;
bool missionReturningHome = false;
MissionCoordinate manualHome = {};
bool manualHomeAvailable = false;
struct PersistedManualHome {
  uint32_t magic;
  int32_t latitudeE7;
  int32_t longitudeE7;
};
constexpr uint32_t MANUAL_HOME_STORAGE_MAGIC = 0x484F4D45UL;
Preferences manualHomePreferences;
bool manualHomePreferencesReady = false;
bool manualHomeButtonReleasedAfterArm = false;
uint32_t manualRthComboStartedMs = 0;
bool manualRthComboTriggered = false;
bool manualReturnHomeActive = false;
uint32_t lastManualReturnUpdateMs = 0;
uint32_t lastManualReturnTelemetryMs = 0;
bool missionCheckpointActive = false;
bool missionStartLocked = false;
bool missionStartBlockedLogged = false;
uint8_t missionStartBlockCode = 0;
uint8_t missionCurrentWaypoint = 0;
uint8_t missionCompletedWaypoints = 0;
int16_t missionReturnWaypointIndex = -1;
char missionLastRunId[37] = "";
float missionDistanceToWaypointMeters = NAN;
float missionDistanceFromHomeMeters = NAN;
uint8_t missionPercentComplete = 0;
uint32_t missionCheckpointStartedMs = 0;
uint32_t lastMissionControlMs = 0;
uint32_t lastMissionUpdateMs = 0;
uint32_t lastMissionTelemetryMs = 0;
String missionRunStatus = "IDLE";
String missionStopReason = "";

uint8_t missionTelemetryStatusCode() {
  if (missionStartBlockCode != 0) return 8;
  if (missionRunStatus == "STARTING") return 1;
  if (missionRunStatus == "MOVING") return 2;
  if (missionRunStatus == "COLLECTING") return 3;
  if (missionRunStatus == "RETURNING_HOME") return 4;
  if (missionRunStatus == "STOPPED") return 5;
  if (missionRunStatus == "COMPLETE") return 6;
  if (missionRunStatus == "RETURNED_HOME") return 7;
  return 0;
}

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
bool otaServiceReady = false;
bool otaUpdateInProgress = false;
// Queue TCP telemetry so a weak Wi-Fi connection cannot stall ELRS control.
constexpr size_t TCP_TELEMETRY_BUFFER_SIZE = 4096;
char tcpTelemetryBuffer[TCP_TELEMETRY_BUFFER_SIZE];
size_t tcpTelemetryBuffered = 0;
WiFiServer sdFileServer(WIFI_SD_FILE_PORT);
WiFiClient sdFileClient;
WiFiUDP udpTelemetry;
IPAddress udpRemoteIp(0, 0, 0, 0);
String tcpCommandBuffer;
File missionUploadFile;
uint32_t missionUploadExpectedBytes = 0;
uint32_t missionUploadExpectedCrc = 0;
uint32_t missionUploadReceivedBytes = 0;
uint16_t udpPacketSequence = 0;
uint16_t udpAudioPacketSequence = 0;
bool ledState = false;
uint32_t lastLedToggleMs = 0;
uint32_t commandFlashUntilMs = 0;
uint32_t lastImuStreamMs = 0;
uint32_t lastRangeStreamMs = 0;
uint32_t lastWaterTempStreamMs = 0;
uint32_t lastWaterChemistryStreamMs = 0;
uint32_t lastPowerStreamMs = 0;
uint32_t lastStateStreamMs = 0;
uint32_t lastMotorOutputStreamMs = 0;
int latestMixerInputLateral = 0;
int latestMixerInputThrust = 0;
int latestMixerInputYaw = 0;
bool latestMixerInputTx12 = false;
uint32_t lastElrsChannelStreamMs = 0;
uint32_t lastGpsStreamMs = 0;
uint32_t lastMagnetometerStreamMs = 0;
uint32_t lastMagnetometerRetryMs = 0;
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
File sdCalibrationFile;
bool elrsCalibrationActive = false;
uint8_t elrsCalibrationStatusCode = 0; // 0=idle, 1=active, 2=stopped, 3=complete
int lastElrsStatusHeading = -1;
uint32_t lastElrsStatusHeadingMs = 0;
uint32_t lastElrsCompactStatusMs = 0;
bool elrsCalibrationArmedOnce = false;
bool elrsCalibrationArmBlockLogged = false;
bool elrsCalibrationTestActive = false;
bool elrsCalibrationFinalTest = false;
uint8_t elrsCalibrationMode = 0; // 0=mixer response, 1=compass motor load
uint8_t elrsCalibrationTestIndex = 0;
uint8_t elrsCalibrationPowerPercent = 0;
uint32_t elrsCalibrationPulseDurationMs = ELRS_CALIBRATION_PULSE_DURATION_DEFAULT_MS;
uint8_t elrsCalibrationCurrentPowerPercent = 0;
uint8_t elrsCalibrationExpectedIndex = 0;
uint8_t elrsCalibrationExpectedPower = 30;
uint8_t elrsCalibrationLastCommand = 0;
uint32_t elrsCalibrationStartedMs = 0;
uint32_t elrsCalibrationTestStartedMs = 0;
uint32_t elrsCalibrationTestEndsMs = 0;
uint32_t elrsCalibrationSampleMs = 0;
uint32_t elrsCalibrationFlushMs = 0;
uint32_t elrsCalibrationSampleCount = 0;
String elrsCalibrationPath;
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
uint32_t lastStandaloneTelemetryMs = 0;
uint32_t lastStandalonePowerSampleMs = 0;
uint32_t lastStandaloneScienceSampleMs = 0;
uint32_t sdLogBootId = 0;
bool standaloneFirstGpsFixRecorded = false;
uint64_t sdCardSizeBytes = 0;
uint64_t sdTotalBytes = 0;
uint64_t sdUsedBytes = 0;
uint16_t latestRangeDistanceMm = 0;
bool latestRangeValid = false;
float latestBatteryVolts = 0.0f;
bool latestBatteryVoltageValid = false;
uint32_t latestBatterySampleMs = 0;
float latestRestingBatteryVolts = 0.0f;
bool latestRestingBatteryVoltageValid = false;
uint32_t latestRestingBatterySampleMs = 0;
float latestCurrentAmps = 0.0f;
bool latestCurrentSampleValid = false;
uint32_t latestCurrentSampleMs = 0;
uint32_t lastMotorStallSampleMs = 0;
uint32_t motorStallNoMovementSinceMs = 0;
bool motorStallGpsAnchorValid = false;
double motorStallGpsAnchorLatitude = 0.0;
double motorStallGpsAnchorLongitude = 0.0;
uint32_t missionLowBatterySinceMs = 0;
ImuReadings standaloneImuReadings = {};
bool standaloneImuValid = false;
float standaloneTdsVoltage = NAN;
float standaloneTurbidityVoltage = NAN;

bool readMpu6050(ImuReadings& readings);
bool setupMpu6050();
bool setupQmc5883();
bool readQmc5883(MagnetometerReadings& readings);
bool calculateTiltHeadingDegrees(const MagnetometerReadings& mag, const ImuReadings& imu,
                                 float& headingDegrees);
String standaloneRunDirectory();
String standaloneBootId();
void emitMagnetometerTelemetry();
bool setupDs18b20();
void updateDs18b20Sensor(uint32_t nowMs);
void emitWaterTemperatureTelemetry();
void emitWaterChemistryTelemetry();
void emitSensorDiagnostics();
bool setupAds1115();
void setupJsnSr04t();
bool readJsnSr04tDistanceMm(uint16_t& distanceMm);
bool setupAudioInput();
bool startAudioCaptureTask();
void processAudioFrames();
void handleControlCommand(String command);
void stopMissionExecution(const String& reason, bool completed = false);
void updateMotorStallProtection(uint32_t now);
bool missionSegmentInsideBoundary(double fromLatitude, double fromLongitude, double toLatitude, double toLongitude);
void updateMissionExecution(uint32_t now);
void updateManualReturnHome(uint32_t now);
bool activeTelemetryHome(MissionCoordinate& home, uint8_t& homeKind);
bool loadManualHomeFromPreferences();
bool saveManualHomeToPreferences(const MissionCoordinate& home);
void beginMissionReturnHome(const String& reason);
String currentScienceTimestamp();
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
bool saveMissionToSd(const char* id, const MissionWaypoint* points, uint8_t count, const MissionMetadata* metadata = nullptr);
bool loadMissionFromSd(const char* id);
bool loadSelectedMissionFromSd();
void emitMissionList();
bool startSdRecorder(const String& startUtc);
bool saveSdRecorder(const String& saveUtc);
void processElrsScienceSwitch(bool manualMode);
int applyManualHeadingAssist(int stickYaw, int scaledYaw, int scaledLateral,
                             int scaledThrust, float powerScale, uint32_t now);
void serviceSdRecorder(uint32_t nowMs);
bool recordStandaloneTelemetrySnapshot();
void recordLogLine(const String& line);
void stopElrsCalibration(const char* reason, bool completed);
bool startElrsCalibrationLog(uint8_t mode, uint8_t maxPower, uint32_t pulseDurationMs);
void processElrsCalibrationCommand(uint32_t now);
void serviceElrsCalibration(uint32_t now);
void writeElrsCalibrationSample(uint32_t now);
float readCurrentSensorVoltage();
float readBatterySystemVoltage();
float currentAmpsFromSensorVoltage(float sensorVoltage);
float updateLatestCurrentSample(float sensorVoltage);
bool updateLatestBatteryVoltage(float volts);
uint32_t missionCrc32File(const char* path, bool& ok);

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

bool validMagnetometerCalibration(float offsetX, float offsetY, float scaleX, float scaleY) {
  return isfinite(offsetX) && isfinite(offsetY) && isfinite(scaleX) && isfinite(scaleY) &&
    fabsf(offsetX) <= 32768.0f && fabsf(offsetY) <= 32768.0f &&
    scaleX >= 0.1f && scaleX <= 10.0f && scaleY >= 0.1f && scaleY <= 10.0f;
}

bool readMagnetometerCalibrationFile(const char* path, float& offsetX, float& offsetY, float& scaleX, float& scaleY) {
  File file = SD.open(path, FILE_READ);
  if (!file) return false;
  String header = file.readStringUntil('\n');
  header.trim();
  String values = file.readStringUntil('\n');
  file.close();
  if (header != "BUOY_NAV_CONFIG_V1") return false;
  if (sscanf(values.c_str(), "MAG_CAL,%f,%f,%f,%f", &offsetX, &offsetY, &scaleX, &scaleY) != 4) return false;
  return validMagnetometerCalibration(offsetX, offsetY, scaleX, scaleY);
}

bool loadNavigationConfigFromSd() {
  if (!sdCardMounted) return false;
  float offsetX = 0.0f, offsetY = 0.0f, scaleX = 1.0f, scaleY = 1.0f;
  if (readMagnetometerCalibrationFile("/config/navigation.cfg", offsetX, offsetY, scaleX, scaleY) ||
      readMagnetometerCalibrationFile("/config/navigation.bak", offsetX, offsetY, scaleX, scaleY)) {
    magnetometerOffsetX = offsetX;
    magnetometerOffsetY = offsetY;
    magnetometerScaleX = scaleX;
    magnetometerScaleY = scaleY;
    magnetometerCalibrationLoaded = true;
    return true;
  }
  magnetometerCalibrationLoaded = false;
  return false;
}

bool saveMagnetometerCalibrationToSd(float offsetX, float offsetY, float scaleX, float scaleY) {
  if (!sdCardMounted || !validMagnetometerCalibration(offsetX, offsetY, scaleX, scaleY)) return false;
  if (!SD.exists("/config") && !SD.mkdir("/config")) return false;
  const char* temporaryPath = "/config/navigation.tmp";
  const char* configPath = "/config/navigation.cfg";
  const char* backupPath = "/config/navigation.bak";
  SD.remove(temporaryPath);
  File file = SD.open(temporaryPath, FILE_WRITE);
  if (!file) return false;
  bool writeOk = file.println("BUOY_NAV_CONFIG_V1") > 0;
  if (writeOk) {
    writeOk = file.printf("MAG_CAL,%.6f,%.6f,%.6f,%.6f\n", offsetX, offsetY, scaleX, scaleY) > 0;
  }
  file.flush();
  file.close();
  float verifyX = 0.0f, verifyY = 0.0f, verifyScaleX = 1.0f, verifyScaleY = 1.0f;
  if (!writeOk || !readMagnetometerCalibrationFile(temporaryPath, verifyX, verifyY, verifyScaleX, verifyScaleY)) {
    SD.remove(temporaryPath);
    return false;
  }
  SD.remove(backupPath);
  const bool hadPrevious = SD.exists(configPath);
  if (hadPrevious && !SD.rename(configPath, backupPath)) {
    SD.remove(temporaryPath);
    return false;
  }
  if (!SD.rename(temporaryPath, configPath)) {
    if (hadPrevious) SD.rename(backupPath, configPath);
    SD.remove(temporaryPath);
    return false;
  }
  SD.remove(backupPath);
  magnetometerOffsetX = offsetX;
  magnetometerOffsetY = offsetY;
  magnetometerScaleX = scaleX;
  magnetometerScaleY = scaleY;
  magnetometerCalibrationLoaded = true;
  missionHeadingCacheValid = false;
  return true;
}

bool readMixerCalibrationFile(const char* path, int8_t values[MIXER_CAL_POWER_STEPS][MIXER_CAL_AXIS_COUNT][MOTOR_COUNT],
                              bool points[MIXER_CAL_POWER_STEPS][MIXER_CAL_AXIS_COUNT]) {
  if (!sdCardMounted) return false;
  File file = SD.open(path, FILE_READ);
  if (!file) return false;
  memset(values, 0, sizeof(int8_t) * MIXER_CAL_POWER_STEPS * MIXER_CAL_AXIS_COUNT * MOTOR_COUNT);
  memset(points, 0, sizeof(bool) * MIXER_CAL_POWER_STEPS * MIXER_CAL_AXIS_COUNT);
  String header = file.readStringUntil('\n');
  header.trim();
  bool valid = header == "BUOY_MIXER_PROFILE_V1";
  while (valid && file.available()) {
    String line = file.readStringUntil('\n');
    line.trim();
    if (!line.length()) continue;
    int power = 0, axis = 0, rear = 0, left = 0, right = 0;
    if (sscanf(line.c_str(), "POINT,%d,%d,%d,%d,%d", &power, &axis, &rear, &left, &right) != 5 ||
        power < 10 || power > 100 || power % 10 != 0 || axis < 0 || axis >= MIXER_CAL_AXIS_COUNT ||
        rear < -100 || rear > 100 || left < -100 || left > 100 || right < -100 || right > 100) {
      valid = false;
      break;
    }
    const int step = power / 10 - 1;
    values[step][axis][0] = static_cast<int8_t>(rear);
    values[step][axis][1] = static_cast<int8_t>(left);
    values[step][axis][2] = static_cast<int8_t>(right);
    points[step][axis] = true;
  }
  file.close();
  return valid;
}

uint8_t mixerCalibrationPointCount(const bool points[MIXER_CAL_POWER_STEPS][MIXER_CAL_AXIS_COUNT]) {
  uint8_t count = 0;
  for (uint8_t step = 0; step < MIXER_CAL_POWER_STEPS; ++step)
    for (uint8_t axis = 0; axis < MIXER_CAL_AXIS_COUNT; ++axis)
      if (points[step][axis]) ++count;
  return count;
}

bool mixerCalibrationComplete() {
  return mixerCalibrationPointCount(mixerCalPointSet) == MIXER_CAL_POWER_STEPS * MIXER_CAL_AXIS_COUNT;
}

bool loadMixerCalibrationFromSd() {
  if (!sdCardMounted) return false;
  mixerCalLoadedFromBackup = false;
  bool valid = readMixerCalibrationFile("/config/propulsion.cfg", mixerCalValues, mixerCalPointSet);
  if (!valid) {
    valid = readMixerCalibrationFile("/config/propulsion.bak", mixerCalValues, mixerCalPointSet);
    mixerCalLoadedFromBackup = valid;
  }
  if (valid) {
    mixerCalProfileLoaded = mixerCalibrationPointCount(mixerCalPointSet) > 0;
    return mixerCalProfileLoaded;
  }
  memset(mixerCalPointSet, 0, sizeof(mixerCalPointSet));
  mixerCalProfileLoaded = false;
  return false;
}

bool saveMixerCalibrationToSd() {
  if (!sdCardMounted || mixerCalibrationPointCount(mixerCalPointSet) == 0) return false;
  if (!SD.exists("/config") && !SD.mkdir("/config")) return false;
  const char* temporaryPath = "/config/propulsion.tmp";
  const char* configPath = "/config/propulsion.cfg";
  const char* backupPath = "/config/propulsion.bak";
  SD.remove(temporaryPath);
  File file = SD.open(temporaryPath, FILE_WRITE);
  if (!file) return false;
  bool ok = file.println("BUOY_MIXER_PROFILE_V1") > 0;
  for (uint8_t step = 0; ok && step < MIXER_CAL_POWER_STEPS; ++step) {
    for (uint8_t axis = 0; ok && axis < MIXER_CAL_AXIS_COUNT; ++axis) {
      if (!mixerCalPointSet[step][axis]) continue;
      ok = file.printf("POINT,%u,%u,%d,%d,%d\n", (step + 1) * 10, axis,
        mixerCalValues[step][axis][0], mixerCalValues[step][axis][1], mixerCalValues[step][axis][2]) > 0;
    }
  }
  file.flush();
  file.close();
  int8_t verifyValues[MIXER_CAL_POWER_STEPS][MIXER_CAL_AXIS_COUNT][MOTOR_COUNT];
  bool verifyPoints[MIXER_CAL_POWER_STEPS][MIXER_CAL_AXIS_COUNT];
  if (!ok || !readMixerCalibrationFile(temporaryPath, verifyValues, verifyPoints) ||
      mixerCalibrationPointCount(verifyPoints) != mixerCalibrationPointCount(mixerCalPointSet)) {
    SD.remove(temporaryPath);
    return false;
  }
  for (uint8_t step = 0; step < MIXER_CAL_POWER_STEPS; ++step) {
    for (uint8_t axis = 0; axis < MIXER_CAL_AXIS_COUNT; ++axis) {
      if (verifyPoints[step][axis] != mixerCalPointSet[step][axis]) { SD.remove(temporaryPath); return false; }
      if (!verifyPoints[step][axis]) continue;
      for (uint8_t motor = 0; motor < MOTOR_COUNT; ++motor) {
        if (verifyValues[step][axis][motor] != mixerCalValues[step][axis][motor]) { SD.remove(temporaryPath); return false; }
      }
    }
  }
  SD.remove(backupPath);
  const bool hadPrevious = SD.exists(configPath);
  if (hadPrevious && !SD.rename(configPath, backupPath)) { SD.remove(temporaryPath); return false; }
  if (!SD.rename(temporaryPath, configPath)) {
    if (hadPrevious) SD.rename(backupPath, configPath);
    SD.remove(temporaryPath);
    return false;
  }
  SD.remove(backupPath);
  mixerCalProfileLoaded = true;
  mixerCalLoadedFromBackup = false;
  return true;
}

void addMixerCalibrationAxis(float input, uint8_t positiveAxis, uint8_t negativeAxis,
                             float output[MOTOR_COUNT]) {
  const uint8_t axis = input >= 0.0f ? positiveAxis : negativeAxis;
  const float magnitude = fabsf(input) * 100.0f;
  if (magnitude <= 0.001f) return;
  const float lowerPosition = magnitude / 10.0f - 1.0f;
  const int lowerStep = max(0, min(static_cast<int>(MIXER_CAL_POWER_STEPS) - 1, static_cast<int>(floorf(lowerPosition))));
  const int upperStep = min(static_cast<int>(MIXER_CAL_POWER_STEPS) - 1, lowerStep + 1);
  const float fraction = lowerPosition < 0.0f ? 0.0f : lowerPosition - lowerStep;
  const float lowPowerScale = magnitude < 10.0f ? magnitude / 10.0f : 1.0f;
  for (int motor = 0; motor < MOTOR_COUNT; ++motor) {
    const float low = mixerCalValues[lowerStep][axis][motor];
    const float high = mixerCalValues[upperStep][axis][motor];
    output[motor] += (low + (high - low) * fraction) * lowPowerScale;
  }
}

bool validMissionId(const char* id) {
  if (id == nullptr || strlen(id) != 36) return false;
  for (size_t i = 0; i < 36; ++i) {
    const char c = id[i];
    if (!((c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') ||
          (c >= 'A' && c <= 'F') || c == '-')) return false;
  }
  return true;
}

String missionFilePath(const char* id, const char* suffix = ".csv") {
  if (!validMissionId(id)) return "";
  return String("/missions/") + id + suffix;
}

String currentMissionFilePath(const char* suffix = ".csv") {
  return String("/missions/current") + suffix;
}

void cleanupObsoleteMissionFiles() {
  if (!sdCardMounted || !SD.exists("/missions")) return;
  while (true) {
    File directory = SD.open("/missions");
    if (!directory || !directory.isDirectory()) {
      if (directory) directory.close();
      return;
    }
    String obsoletePath = "";
    File entry;
    while ((entry = directory.openNextFile())) {
      String filename = String(entry.name());
      int slash = filename.lastIndexOf('/');
      if (slash >= 0) filename = filename.substring(slash + 1);
      entry.close();
      if (!filename.endsWith(".csv")) continue;
      String id = filename.substring(0, filename.length() - 4);
      if (validMissionId(id.c_str())) {
        obsoletePath = missionFilePath(id.c_str());
        break;
      }
    }
    directory.close();
    if (obsoletePath.length() == 0) break;
    if (!SD.remove(obsoletePath.c_str())) break;
  }
}

bool readMissionFile(const char* path, MissionWaypoint* points, uint8_t& count, char* idOut, MissionMetadata* metadataOut = nullptr) {
  File file = SD.open(path, FILE_READ);
  if (!file) return false;
  String header = file.readStringUntil('\n');
  header.trim();
  char parsedId[37] = "";
  unsigned int parsedCount = 0;
  unsigned int boundaryCount = 0, hasHome = 0, hasEnd = 0;
  MissionMetadata parsedMetadata;
  const bool version4 = header.startsWith("BUOY_MISSION_V4,");
  const bool version3 = version4 || header.startsWith("BUOY_MISSION_V3,");
  const bool version2 = version3 || header.startsWith("BUOY_MISSION_V2,");
  unsigned int linkLossAction = 1, boundaryBreachAction = 1, lowBatteryAction = 1, batteryThresholdPercent = 20;
  const int headerFields = version4
    ? sscanf(header.c_str(), "BUOY_MISSION_V4,%36[^,],%u,%u,%u,%u,%u,%u,%u,%u", parsedId, &parsedCount, &boundaryCount, &hasHome, &hasEnd, &linkLossAction, &boundaryBreachAction, &lowBatteryAction, &batteryThresholdPercent)
    : version2
    ? sscanf(header.c_str(), version3 ? "BUOY_MISSION_V3,%36[^,],%u,%u,%u,%u" : "BUOY_MISSION_V2,%36[^,],%u,%u,%u,%u", parsedId, &parsedCount, &boundaryCount, &hasHome, &hasEnd)
    : sscanf(header.c_str(), "BUOY_MISSION_V1,%36[^,],%u", parsedId, &parsedCount);
  if (headerFields != (version4 ? 9 : (version2 ? 5 : 2)) ||
      !validMissionId(parsedId) || parsedCount == 0 || parsedCount > MAX_MISSION_WAYPOINTS) {
    file.close();
    return false;
  }
  if (boundaryCount > MAX_MISSION_BOUNDARY_POINTS || hasHome > 1 || hasEnd > 1 ||
      linkLossAction > 1 || boundaryBreachAction > 1 || lowBatteryAction > 1 ||
      batteryThresholdPercent < 5 || batteryThresholdPercent > 80) { file.close(); return false; }
  if (version4) {
    parsedMetadata.linkLossAction = static_cast<uint8_t>(linkLossAction);
    parsedMetadata.boundaryBreachAction = static_cast<uint8_t>(boundaryBreachAction);
    parsedMetadata.lowBatteryAction = static_cast<uint8_t>(lowBatteryAction);
    parsedMetadata.batteryThresholdPercent = static_cast<uint8_t>(batteryThresholdPercent);
  }
  if (version2) {
    parsedMetadata.hasHome = hasHome != 0;
    parsedMetadata.hasEnd = hasEnd != 0;
    for (uint8_t kind = 0; kind < 2; ++kind) {
      if ((kind == 0 && !parsedMetadata.hasHome) || (kind == 1 && !parsedMetadata.hasEnd)) continue;
      String line = file.readStringUntil('\n'); line.trim();
      float lat = 0, lon = 0;
      if (sscanf(line.c_str(), kind == 0 ? "HOME,%f,%f" : "END,%f,%f", &lat, &lon) != 2 ||
          !isfinite(lat) || !isfinite(lon) || lat < -90 || lat > 90 || lon < -180 || lon > 180) { file.close(); return false; }
      if (kind == 0) parsedMetadata.home = {lat, lon}; else parsedMetadata.end = {lat, lon};
    }
    for (unsigned int i = 0; i < boundaryCount; ++i) {
      String line = file.readStringUntil('\n'); line.trim();
      unsigned int index = 0; float lat = 0, lon = 0;
      if (sscanf(line.c_str(), "BOUNDARY,%u,%f,%f", &index, &lat, &lon) != 3 || index != i ||
          !isfinite(lat) || !isfinite(lon) || lat < -90 || lat > 90 || lon < -180 || lon > 180) { file.close(); return false; }
      parsedMetadata.boundary[i] = {lat, lon};
    }
    parsedMetadata.boundaryCount = static_cast<uint8_t>(boundaryCount);
  }
  for (unsigned int i = 0; i < parsedCount; ++i) {
    String line = file.readStringUntil('\n');
    line.trim();
    float latitude = 0.0f, longitude = 0.0f; unsigned int measurementMask = 0, collectData = 0, holdSeconds = 0, speedPercent = 0;
    const int pointFields = version3
      ? sscanf(line.c_str(), "POINT,%f,%f,%u,%u,%u,%u", &latitude, &longitude, &measurementMask, &collectData, &holdSeconds, &speedPercent)
      : (version2 ? sscanf(line.c_str(), "POINT,%f,%f,%u,%u,%u", &latitude, &longitude, &measurementMask, &holdSeconds, &speedPercent)
      : sscanf(line.c_str(), "%f,%f", &latitude, &longitude));
    if (pointFields != (version3 ? 6 : (version2 ? 5 : 2)) || measurementMask > 31 || collectData > 1 || holdSeconds > 3600 || speedPercent > 100 ||
        !isfinite(latitude) || !isfinite(longitude) || latitude < -90.0f ||
        latitude > 90.0f || longitude < -180.0f || longitude > 180.0f) {
      file.close();
      return false;
    }
    points[i] = {latitude, longitude, static_cast<uint16_t>(measurementMask), version3 ? collectData != 0 : (version2 ? holdSeconds > 0 : false), static_cast<uint16_t>(holdSeconds), static_cast<uint8_t>(version2 ? speedPercent : 50)};
  }
  String extra = file.readStringUntil('\n');
  extra.trim();
  file.close();
  if (extra.length() != 0) return false;
  count = static_cast<uint8_t>(parsedCount);
  if (idOut) strlcpy(idOut, parsedId, 37);
  if (metadataOut) *metadataOut = parsedMetadata;
  return true;
}

bool saveMissionToSd(const char* id, const MissionWaypoint* points, uint8_t count, const MissionMetadata* metadata) {
  if (!sdCardMounted || !validMissionId(id) || points == nullptr || count == 0 || count > MAX_MISSION_WAYPOINTS) return false;
  MissionMetadata emptyMetadata;
  if (!metadata) metadata = &emptyMetadata;
  if (metadata->boundaryCount > MAX_MISSION_BOUNDARY_POINTS) return false;
  if (!SD.exists("/missions") && !SD.mkdir("/missions")) return false;
  const String missionPath = currentMissionFilePath();
  const String backupPath = currentMissionFilePath(".bak");
  const char* temporaryPath = "/missions/upload.tmp";
  SD.remove(temporaryPath);
  File file = SD.open(temporaryPath, FILE_WRITE);
  if (!file) return false;
  bool writeOk = file.println(String("BUOY_MISSION_V4,") + id + "," + String(count) + "," + String(metadata->boundaryCount) + "," + String(metadata->hasHome ? 1 : 0) + "," + String(metadata->hasEnd ? 1 : 0) + "," + String(metadata->linkLossAction) + "," + String(metadata->boundaryBreachAction) + "," + String(metadata->lowBatteryAction) + "," + String(metadata->batteryThresholdPercent)) > 0;
  if (writeOk && metadata->hasHome) writeOk = file.println(String("HOME,") + String(metadata->home.latitude, 7) + "," + String(metadata->home.longitude, 7)) > 0;
  if (writeOk && metadata->hasEnd) writeOk = file.println(String("END,") + String(metadata->end.latitude, 7) + "," + String(metadata->end.longitude, 7)) > 0;
  for (uint8_t i = 0; writeOk && i < metadata->boundaryCount; ++i)
    writeOk = file.println(String("BOUNDARY,") + String(i) + "," + String(metadata->boundary[i].latitude, 7) + "," + String(metadata->boundary[i].longitude, 7)) > 0;
  for (uint8_t i = 0; writeOk && i < count; ++i) {
    writeOk = file.println(String("POINT,") + String(points[i].latitude, 7) + "," + String(points[i].longitude, 7) + "," + String(points[i].measurementMask) + "," + String(points[i].collectData ? 1 : 0) + "," + String(points[i].holdSeconds) + "," + String(points[i].speedPercent)) > 0;
  }
  file.flush();
  file.close();
  if (!writeOk) {
    SD.remove(temporaryPath);
    return false;
  }
  MissionWaypoint verified[MAX_MISSION_WAYPOINTS] = {};
  uint8_t verifiedCount = 0;
  char verifiedId[37] = "";
  MissionMetadata verifiedMetadata;
  if (!readMissionFile(temporaryPath, verified, verifiedCount, verifiedId, &verifiedMetadata) ||
      verifiedCount != count || strcmp(verifiedId, id) != 0) {
    SD.remove(temporaryPath);
    return false;
  }
  if (verifiedMetadata.hasHome != metadata->hasHome || verifiedMetadata.hasEnd != metadata->hasEnd || verifiedMetadata.boundaryCount != metadata->boundaryCount ||
      verifiedMetadata.linkLossAction != metadata->linkLossAction || verifiedMetadata.boundaryBreachAction != metadata->boundaryBreachAction ||
      verifiedMetadata.lowBatteryAction != metadata->lowBatteryAction || verifiedMetadata.batteryThresholdPercent != metadata->batteryThresholdPercent ||
      (metadata->hasHome && (fabsf(verifiedMetadata.home.latitude - metadata->home.latitude) > 0.000001f || fabsf(verifiedMetadata.home.longitude - metadata->home.longitude) > 0.000001f)) ||
      (metadata->hasEnd && (fabsf(verifiedMetadata.end.latitude - metadata->end.latitude) > 0.000001f || fabsf(verifiedMetadata.end.longitude - metadata->end.longitude) > 0.000001f))) { SD.remove(temporaryPath); return false; }
  for (uint8_t i = 0; i < metadata->boundaryCount; ++i) {
    if (fabsf(verifiedMetadata.boundary[i].latitude - metadata->boundary[i].latitude) > 0.000001f ||
        fabsf(verifiedMetadata.boundary[i].longitude - metadata->boundary[i].longitude) > 0.000001f) { SD.remove(temporaryPath); return false; }
  }
  for (uint8_t i = 0; i < count; ++i) {
    if (fabsf(verified[i].latitude - points[i].latitude) > 0.000001f ||
        fabsf(verified[i].longitude - points[i].longitude) > 0.000001f ||
        verified[i].measurementMask != points[i].measurementMask || verified[i].collectData != points[i].collectData ||
        verified[i].holdSeconds != points[i].holdSeconds || verified[i].speedPercent != points[i].speedPercent) {
      SD.remove(temporaryPath);
      return false;
    }
  }

  SD.remove(backupPath.c_str());
  const bool hadPrevious = SD.exists(missionPath.c_str());
  if (hadPrevious && !SD.rename(missionPath.c_str(), backupPath.c_str())) {
    SD.remove(temporaryPath);
    return false;
  }
  if (!SD.rename(temporaryPath, missionPath.c_str())) {
    if (hadPrevious) SD.rename(backupPath.c_str(), missionPath.c_str());
    SD.remove(temporaryPath);
    return false;
  }
  return true;
}

bool loadMissionFromSd(const char* id) {
  if (!sdCardMounted) return false;
  const String missionPath = currentMissionFilePath();
  const String backupPath = currentMissionFilePath(".bak");
  MissionWaypoint loaded[MAX_MISSION_WAYPOINTS] = {};
  uint8_t loadedCount = 0;
  char loadedId[37] = "";
  MissionMetadata loadedMetadata;
  if (readMissionFile(missionPath.c_str(), loaded, loadedCount, loadedId, &loadedMetadata) && strcmp(loadedId, id) == 0) {
    memcpy(missionWaypoints, loaded, loadedCount * sizeof(MissionWaypoint));
    missionWaypointCount = loadedCount;
    missionMetadata = loadedMetadata;
    strlcpy(missionId, loadedId, sizeof(missionId));
    SD.remove(backupPath.c_str());
    cleanupObsoleteMissionFiles();
    return true;
  }
  if (readMissionFile(backupPath.c_str(), loaded, loadedCount, loadedId, &loadedMetadata) && strcmp(loadedId, id) == 0) {
    memcpy(missionWaypoints, loaded, loadedCount * sizeof(MissionWaypoint));
    missionWaypointCount = loadedCount;
    missionMetadata = loadedMetadata;
    strlcpy(missionId, loadedId, sizeof(missionId));
    SD.remove(missionPath.c_str());
    SD.rename(backupPath.c_str(), missionPath.c_str());
    cleanupObsoleteMissionFiles();
    return true;
  }
  missionWaypointCount = 0;
  missionMetadata = MissionMetadata();
  missionId[0] = '\0';
  return false;
}

bool deleteMissionFromSd(const char* id) {
  if (!sdCardMounted || !validMissionId(id)) return false;
  if (runningMissionId[0] && strcmp(runningMissionId, id) == 0) return false;
  const String path = currentMissionFilePath();
  const String backupPath = currentMissionFilePath(".bak");
  if (!SD.exists(path.c_str()) && !SD.exists(backupPath.c_str())) return false;
  MissionWaypoint currentPoints[MAX_MISSION_WAYPOINTS] = {};
  uint8_t currentCount = 0;
  char currentId[37] = "";
  bool matchesCurrent = readMissionFile(path.c_str(), currentPoints, currentCount, currentId) && strcmp(currentId, id) == 0;
  if (!matchesCurrent) {
    matchesCurrent = readMissionFile(backupPath.c_str(), currentPoints, currentCount, currentId) && strcmp(currentId, id) == 0;
  }
  if (!matchesCurrent) return false;
  if (SD.exists(path.c_str()) && !SD.remove(path.c_str())) return false;
  if (SD.exists(backupPath.c_str()) && !SD.remove(backupPath.c_str())) return false;

  if (missionId[0] && strcmp(missionId, id) == 0) {
    missionWaypointCount = 0;
    missionMetadata = MissionMetadata();
    missionId[0] = '\0';
  }
  File selection = SD.open("/missions/selected.txt", FILE_READ);
  if (selection) {
    String selectedId = selection.readStringUntil('\n');
    selection.close();
    selectedId.trim();
    if (selectedId.equalsIgnoreCase(id) && SD.exists("/missions/selected.txt")) {
      SD.remove("/missions/selected.txt");
    }
  }
  return true;
}

bool persistMissionSelection(const char* id) {
  if (!sdCardMounted || !validMissionId(id)) return false;
  if (!SD.exists("/missions") && !SD.mkdir("/missions")) return false;
  SD.remove("/missions/selected.tmp");
  File file = SD.open("/missions/selected.tmp", FILE_WRITE);
  if (!file) return false;
  const bool written = file.println(id) > 0;
  file.flush();
  file.close();
  if (!written) return false;
  SD.remove("/missions/selected.txt");
  if (!SD.rename("/missions/selected.tmp", "/missions/selected.txt")) {
    SD.remove("/missions/selected.tmp");
    return false;
  }
  return true;
}

bool loadSelectedMissionFromSd() {
  if (!sdCardMounted) return false;
  File file = SD.open("/missions/selected.txt", FILE_READ);
  if (!file) return false;
  String selectedId = file.readStringUntil('\n');
  file.close();
  selectedId.trim();
  return loadMissionFromSd(selectedId.c_str());
}

bool migrateSelectedMissionCollectionFromSd() {
  if (!sdCardMounted) return false;
  File selection = SD.open("/missions/selected.txt", FILE_READ);
  if (!selection) return false;
  String selectedId = selection.readStringUntil('\n');
  selection.close();
  selectedId.trim();
  if (!validMissionId(selectedId.c_str())) return false;
  const String oldPath = missionFilePath(selectedId.c_str());
  MissionWaypoint points[MAX_MISSION_WAYPOINTS] = {};
  uint8_t count = 0;
  char fileId[37] = "";
  if (!readMissionFile(oldPath.c_str(), points, count, fileId) || strcmp(fileId, selectedId.c_str()) != 0) return false;
  if (!saveMissionToSd(fileId, points, count) ||
      !persistMissionSelection(fileId) || !loadMissionFromSd(fileId)) return false;
  SD.remove(currentMissionFilePath(".bak").c_str());
  cleanupObsoleteMissionFiles();
  return true;
}

bool migrateLegacyMissionFromSd() {
  const char* legacyId = "00000000-0000-4000-8000-000000000001";
  if (!sdCardMounted || !SD.exists("/mission.txt")) return false;
  MissionWaypoint points[MAX_MISSION_WAYPOINTS] = {};
  uint8_t count = 0;
  File file = SD.open("/mission.txt", FILE_READ);
  if (!file) return false;
  String header = file.readStringUntil('\n');
  header.trim();
  unsigned int parsedCount = 0;
  if (sscanf(header.c_str(), "BUOY_MISSION_V1,%u", &parsedCount) != 1 ||
      parsedCount == 0 || parsedCount > MAX_MISSION_WAYPOINTS) {
    file.close();
    return false;
  }
  for (unsigned int i = 0; i < parsedCount; ++i) {
    String line = file.readStringUntil('\n');
    line.trim();
    float latitude = 0.0f, longitude = 0.0f;
    if (sscanf(line.c_str(), "%f,%f", &latitude, &longitude) != 2 ||
        !isfinite(latitude) || !isfinite(longitude) || latitude < -90.0f ||
        latitude > 90.0f || longitude < -180.0f || longitude > 180.0f) {
      file.close();
      return false;
    }
    points[i] = {latitude, longitude};
  }
  file.close();
  if (!saveMissionToSd(legacyId, points, static_cast<uint8_t>(parsedCount)) ||
      !persistMissionSelection(legacyId) || !loadMissionFromSd(legacyId)) return false;
  SD.remove(currentMissionFilePath(".bak").c_str());
  SD.rename("/mission.txt", "/mission.legacy.bak");
  cleanupObsoleteMissionFiles();
  return true;
}

void emitMissionList() {
  if (!sdCardMounted) {
    emitLine("ERR MISSION SD UNAVAILABLE");
    return;
  }
  MissionWaypoint points[MAX_MISSION_WAYPOINTS] = {};
  uint8_t pointCount = 0;
  char currentId[37] = "";
  bool hasMission = readMissionFile(currentMissionFilePath().c_str(), points, pointCount, currentId);
  if (!hasMission) hasMission = readMissionFile(currentMissionFilePath(".bak").c_str(), points, pointCount, currentId);
  emitLine(String("ACK MISSION LIST ") + (hasMission ? "1" : "0"));
  if (hasMission) emitLine("TEL MISSION ITEM " + String(currentId) + " " + String(pointCount));
  emitLine("TEL MISSION ACTIVE " + String(runningMissionId[0] ? runningMissionId : "NONE") +
    " LOADED " + String(hasMission ? currentId : "NONE"));
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
  sdScienceFile.println(F("timestamp_us,mission_waypoint,mission_collecting,measurement_mask,imu_valid,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,imu_temp_c,water_temp_c,range_valid,range_mm,ads_current_v,ads_battery_v,tds_v,turbidity_probe_v"));
  sdTelemetryFile.println(F("timestamp_us,gps_valid,gps_date_ddmmyy,gps_time_hhmmsscc,latitude,longitude,control_mode,mission_waypoint,mission_collecting,measurement_mask,hold,motors_enabled,rear_drive,left_drive,right_drive,battery_v,current_a,audio_frames_dropped,sd_available,sd_mount_failures,sd_write_failures,sd_card_mib,sd_total_mib,sd_used_mib,sd_free_mib,sd_audio_bytes,sd_audio_buffer_bytes,sd_audio_queue_depth"));
  File manifest = SD.open((sdSessionDirectory + "/manifest.txt").c_str(), FILE_WRITE);
  if (manifest) {
    const bool missionStartedSession = missionExecutionActive && runningMissionId[0] != '\0';
    manifest.println(String("boot_id=") + standaloneBootId());
    manifest.println(String("startup_logs=") + standaloneRunDirectory());
    manifest.println("firmware_version=" + firmwareVersionString());
    manifest.println("firmware_major=" + String(kFirmwareVersionMajor));
    manifest.println("firmware_minor=" + String(kFirmwareVersionMinor));
    manifest.println("firmware_patch=" + String(kFirmwareVersionPatch));
    manifest.println("firmware_build_date=" + String(kFirmwareBuildDate));
    manifest.println(String("session_origin=") + (missionStartedSession ? "mission_auto" : "manual"));
    manifest.println(String("mission_id=") + (missionStartedSession ? String(runningMissionId) : String("NONE")));
    if (missionStartedSession) {
      manifest.println(String("mission_run_id=") + sdSessionDirectory.substring(1));
      manifest.println(String("mission_started_utc=") + (startUtc.startsWith("00000000T") ? String("UNKNOWN") : startUtc));
      manifest.println(String("mission_home=") + String(missionMetadata.home.latitude, 7) + "," + String(missionMetadata.home.longitude, 7));
      manifest.println(String("mission_has_end=") + String(missionMetadata.hasEnd ? 1 : 0));
      if (missionMetadata.hasEnd) {
        manifest.println(String("mission_end=") + String(missionMetadata.end.latitude, 7) + "," + String(missionMetadata.end.longitude, 7));
      }
      manifest.println(String("mission_boundary_count=") + String(missionMetadata.boundaryCount));
      for (uint8_t i = 0; i < missionMetadata.boundaryCount; ++i) {
        const MissionCoordinate& point = missionMetadata.boundary[i];
        manifest.println(String("mission_boundary_") + String(i + 1) + "=" + String(point.latitude, 7) + "," + String(point.longitude, 7));
      }
      manifest.println(String("mission_waypoint_count=") + String(missionWaypointCount));
      manifest.println(F("mission_measurement_mask bits: water_temperature=1, depth=2, conductivity=4, turbidity=8, air_temperature=16"));
      for (uint8_t i = 0; i < missionWaypointCount; ++i) {
        const MissionWaypoint& point = missionWaypoints[i];
        manifest.println(String("mission_waypoint_") + String(i + 1) + "=" +
          String(point.latitude, 7) + "," + String(point.longitude, 7) + "," +
          String(point.measurementMask) + "," + String(point.collectData ? 1 : 0) + "," +
          String(point.holdSeconds) + "," + String(point.speedPercent));
      }
      manifest.println(F("audio_policy=recorded only during stationary mission collection holds; excluded during transit"));
    } else {
      manifest.println(F("mission_run_id=NONE"));
      manifest.println(F("mission_started_utc=NONE"));
    }
    if (startUtc.startsWith("00000000T")) {
      manifest.println(F("start_utc=UNKNOWN"));
      manifest.println("start_elapsed_ms=" + String(millis()));
    } else {
      manifest.println("start_utc=" + startUtc);
    }
    manifest.println(F("status=recording"));
    manifest.println(F("timestamp_us is the 64-bit monotonic ESP timer; science.csv and telemetry.csv include current mission waypoint, collection state, and measurement mask."));
    if (missionStartedSession) {
      manifest.println(String("mission_collection_sample_interval_ms=") + String(SD_SCIENCE_INTERVAL_MS));
      manifest.println(F("mission session opens once at mission start and stays open through transit, collection holds, and return home; it closes at mission stop or completion."));
    }
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
  if (saveUtc.startsWith("00000000T")) {
    manifest.println(F("saved_utc=UNKNOWN"));
    manifest.println("saved_elapsed_ms=" + String(millis()));
  } else {
    manifest.println("saved_utc=" + saveUtc);
  }
  manifest.println(F("status=complete"));
  manifest.close();
  return true;
}

void stopRecordingForManualLinkLoss() {
  if (scienceExperimentState != 1) return;
  audioRecordingEnabled = false;
  const String stoppedAt = "LINKLOSS-" + String(millis());
  if (!flushAudioWriteBuffer()) return;
  updateWavHeader();
  sdAudioFile.flush();
  sdAudioIndexFile.flush();
  sdScienceFile.flush();
  sdTelemetryFile.flush();
  File manifest = SD.open((sdSessionDirectory + "/manifest.txt").c_str(), FILE_APPEND);
  if (manifest) {
    manifest.println("stopped_at=" + stoppedAt);
    manifest.println(F("stop_reason=manual_elrs_link_loss"));
    manifest.close();
  }
  scienceExperimentState = 2;
  saveSdRecorder(stoppedAt);
}

String standaloneBootId() {
  char bootId[9];
  snprintf(bootId, sizeof(bootId), "%08lx", static_cast<unsigned long>(sdLogBootId));
  return String(bootId);
}

String standaloneRunDirectory() {
  return String("/logs/run-") + standaloneBootId();
}

String standaloneLogPath(const char* stem, const char* extension) {
  return standaloneRunDirectory() + "/" + stem + extension;
}

bool ensureStandaloneLogDirectory() {
  if (!sdCardMounted) return false;
  if (!SD.exists("/logs") && !SD.mkdir("/logs")) {
    ++sdWriteFailures;
    return false;
  }
  const String runDirectory = standaloneRunDirectory();
  if (SD.exists(runDirectory.c_str())) return true;
  if (!SD.mkdir(runDirectory.c_str())) {
    ++sdWriteFailures;
    return false;
  }
  File manifest = SD.open((runDirectory + "/run.txt").c_str(), FILE_WRITE);
  if (manifest) {
    manifest.println(String("boot_id=") + standaloneBootId());
    manifest.println(String("firmware_version=") + firmwareVersionString());
    manifest.println(String("magnetometer_calibrated=") + String(magnetometerCalibrationLoaded ? 1 : 0));
    manifest.println(String("mag_offset_x=") + String(magnetometerOffsetX, 4));
    manifest.println(String("mag_offset_y=") + String(magnetometerOffsetY, 4));
    manifest.println(String("mag_scale_x=") + String(magnetometerScaleX, 6));
    manifest.println(String("mag_scale_y=") + String(magnetometerScaleY, 6));
    manifest.println(F("telemetry=telemetry.csv"));
    manifest.println(F("events=system.csv"));
    manifest.println(F("time_source=GPS date and time columns in telemetry.csv"));
    manifest.close();
  } else {
    ++sdWriteFailures;
  }
  return true;
}

void printCsvQuoted(File& file, const String& value) {
  file.print('"');
  for (size_t i = 0; i < value.length(); ++i) {
    const char c = value.charAt(i);
    if (c == '"') file.print('"');
    if (c == '\r' || c == '\n') file.print(' ');
    else file.print(c);
  }
  file.print('"');
}

void recordLogLine(const String& line) {
  if (!ensureStandaloneLogDirectory()) return;
  const String path = standaloneLogPath("system", ".csv");
  File file = SD.open(path.c_str(), FILE_APPEND);
  if (!file) {
    ++sdWriteFailures;
    return;
  }
  if (file.size() == 0) {
    file.println(F("gps_date_ddmmyy,gps_time_hhmmsscc,uptime_ms,boot_id,event"));
  }
  if (gps.date.isValid()) file.print(gps.date.value());
  file.print(',');
  if (gps.time.isValid()) file.print(gps.time.value());
  file.print(',');
  file.print(millis());
  file.print(',');
  file.print(standaloneBootId());
  file.print(',');
  printCsvQuoted(file, line);
  file.println();
  file.flush();
  if (file.getWriteError()) ++sdWriteFailures;
  file.close();
}

void emitLine(const String& line) {
  if (!controlClientConnected()) return;
  const size_t length = line.length();
  if (length + 1 > TCP_TELEMETRY_BUFFER_SIZE - tcpTelemetryBuffered) return;
  memcpy(tcpTelemetryBuffer + tcpTelemetryBuffered, line.c_str(), length);
  tcpTelemetryBuffered += length;
  tcpTelemetryBuffer[tcpTelemetryBuffered++] = '\n';
}

void reportMissionStartBlock(uint8_t code, const char* reason) {
  if (missionStartBlockCode == code) return;
  missionStartBlockCode = code;
  missionStartBlockedLogged = true;
  emitLine(String("TEL EVENT MISSION_START BLOCKED ") + String(code) + " " + reason);
  recordLogLine(String("MISSION START BLOCKED ") + String(code) + " " + reason);
  elrs.sendScienceValue("MIS_STATE", 8);
  elrs.sendScienceValue("MIS_BLOCK", code);
}

void flushTcpTelemetry() {
  if (!tcpTelemetryBuffered || !controlClientConnected()) return;
  const int socketFd = tcpClient.fd();
  if (socketFd < 0) return;
  const size_t writeLength = min(tcpTelemetryBuffered, static_cast<size_t>(512));
  const int written = send(socketFd, tcpTelemetryBuffer, writeLength, MSG_DONTWAIT);
  if (written <= 0) return;
  tcpTelemetryBuffered -= static_cast<size_t>(written);
  if (tcpTelemetryBuffered) {
    memmove(tcpTelemetryBuffer, tcpTelemetryBuffer + written, tcpTelemetryBuffered);
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
  String line = "TEL MOTOR OUT";
  for (int i = 0; i < MOTOR_COUNT; i++) {
    line += " " + String(driveValues[i]) + " " + String(appliedMotorPwm[i]);
  }
  emitLine(line);
  emitLine(String("TEL MIX INPUT ") + String(latestMixerInputLateral) + " " +
    String(latestMixerInputThrust) + " " + String(latestMixerInputYaw) + " " +
    String(latestMixerInputTx12 ? 1 : 0));
}

void sendStatusSnapshot() {
  emitLine(firmwareVersionStatusLine());
  emitLine(String("TEL STATUS TELEMETRY_MODE ") +
    (telemetryMode == TELEMETRY_MODE_MAVLINK ? "MAVLINK" : "WIFI"));
  emitLine(String("TEL STATUS CONTROL_SOURCE ") +
    (controlSource == CONTROL_SOURCE_ELRS ? "TX12" :
     controlSource == CONTROL_SOURCE_WIFI ? "WIFI" : "NONE"));
  emitLine(String("TEL STATUS ELRS_ARM ") + (elrsArmed ? "ARMED" : "DISARMED"));
  emitLine(manualHomeAvailable
    ? String("TEL STATUS MANUAL_HOME SET ") + String(manualHome.latitude, 7) + " " + String(manualHome.longitude, 7)
    : String("TEL STATUS MANUAL_HOME UNSET"));
  MissionCoordinate activeHome = {};
  uint8_t activeHomeKind = 0;
  if (activeTelemetryHome(activeHome, activeHomeKind)) {
    emitLine(String("TEL STATUS ACTIVE_HOME ") + (activeHomeKind == 1 ? "MANUAL " : "MISSION ") +
      String(activeHome.latitude, 7) + " " + String(activeHome.longitude, 7));
  } else {
    emitLine("TEL STATUS ACTIVE_HOME NONE");
  }
  emitLine(String("TEL STATUS RECOVERY ") + (manualLinkLossStop ? "MANUAL_STOP" : "READY"));
  emitLine(String("TEL STATUS MAGNETOMETER ") + (magnetometerAvailable ? "READY" : "UNAVAILABLE"));
  emitLine(String("TEL STATUS MAGCAL ") + (magnetometerCalibrationLoaded ? "LOADED SD" : "DEFAULT"));
  emitLine(String("TEL STATUS ELRS_FAILSAFE ") + lastElrsFailsafeStatus);
  emitLine(String("TEL STATUS ELRS BYTES ") + String(elrs.receivedBytes()) +
    " RADIO " + String(elrs.validRadioStatus()) +
    " RC " + String(elrs.validRcFrames()) +
    " CRCERR " + String(elrs.crcErrors()));
  const char* rcReject = "NONE";
  switch (elrs.lastRcRejectCode()) {
    case 1: rcReject = "CRC"; break;
    case 2: rcReject = "SHORT"; break;
    case 3: rcReject = "TARGET"; break;
    case 4: rcReject = "CHANNEL"; break;
  }
  emitLine(String("TEL STATUS ELRS_RC_DIAG OVERRIDE ") + String(elrs.rcOverrideFrames()) +
    " CRC_OK " + String(elrs.rcOverrideCrcOk()) +
    " CRC_BAD " + String(elrs.rcOverrideCrcBad()) +
    " SHORT " + String(elrs.rcOverrideShort()) +
    " TARGET_BAD " + String(elrs.rcOverrideTargetReject()) +
    " CHANNEL_BAD " + String(elrs.rcOverrideChannelReject()) +
    " ACCEPTED " + String(elrs.validRcFrames()) +
    " LAST_REJECT " + rcReject +
    " TARGET " + String(elrs.lastRcTargetSystem()) + " " + String(elrs.lastRcTargetComponent()) +
    " BAD_CH " + String(elrs.lastRcInvalidChannel()) + " VALUE " + String(elrs.lastRcInvalidValue()));
  emitLine(String("TEL STATUS SCIENCE ") +
    (scienceExperimentState == 1 ? "RECORDING " : (scienceExperimentState == 2 ? "STOPPED " : "IDLE ")) +
    (sdSessionDirectory.length() ? sdSessionDirectory : "NONE"));
  // In Backpack mode the Wi-Fi heartbeat reports only control and safety.
  // Power, position, motor state and channels travel over MAVLink instead.
  if (telemetryMode == TELEMETRY_MODE_MAVLINK) return;
  const uint32_t linkAgeMs = elrs.lastLinkMs() == 0 ? UINT32_MAX : millis() - elrs.lastLinkMs();
  emitLine(String("TEL STATUS ELRS_LINK LQ ") + String(elrs.linkQuality()) +
    " AGE_MS " + (linkAgeMs == UINT32_MAX ? String("UNKNOWN") : String(linkAgeMs)) +
    " RSSI_DBM " + String(elrs.rssiDbm()) +
    " SNR_DB " + String(elrs.snrDb()));
  String channelLine = "TEL STATUS ELRS_CHANNELS";
  for (uint8_t channel = 0; channel < 16; ++channel) {
    channelLine += " " + String(elrs.channel(channel));
  }
  emitLine(channelLine);
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
  emitLine(String("TEL STATUS MISSION_CONTROL ") +
    (missionModeRequested ? (missionExecutionActive ? missionRunStatus :
      (missionStartLocked ? String("MISSION_LOCKED") :
       (missionStartBlockCode ? String("MISSION_BLOCKED_") + String(missionStartBlockCode) : String("MISSION_READY")))) :
     manualAssistRequested ? (manualAssistHeadingAvailable
       ? String("MANUAL_ASSIST") : String("MANUAL_ASSIST_NO_HEADING")) : String("MANUAL")));
  emitLine(String("TEL STATUS MISSION ") + String(missionWaypointCount) +
    " UPLOAD " + (missionUploadInProgress ? "RECEIVING" : "READY") +
    " LOADED " + String(missionId[0] ? missionId : "NONE") +
    " RUNNING " + String(runningMissionId[0] ? runningMissionId : "NONE"));

  if (gps.location.isValid()) {
    emitLine("TEL STATUS POS " + String(gps.location.lat(), 6) + " " + String(gps.location.lng(), 6));
  } else {
    emitLine("TEL STATUS POS UNKNOWN UNKNOWN");
  }
  emitLine(String("TEL STATUS GPS ") +
    (gps.satellites.isValid() ? String(gps.satellites.value()) : String("UNKNOWN")) + " " +
    (gps.hdop.isValid() ? String(gps.hdop.hdop(), 1) : String("UNKNOWN")) + " " +
    (gps.location.isValid() ? "FIX" : "NOFIX"));

  if (targetLocationSet) {
    emitLine("TEL STATUS TARGET " + String(targetLatitude, 6) + " " + String(targetLongitude, 6));
  } else {
    emitLine("TEL STATUS TARGET UNKNOWN UNKNOWN");
  }

  emitLine(holdPositionEnabled ? "TEL STATUS HOLD ON" : "TEL STATUS HOLD OFF");
  emitLine(buildSdStatusLine());
  emitLine(String("TEL STATUS STREAM AUDIO ") + (audioStreamEnabled ? "ON" : "OFF"));
  emitLine(String("TEL STATUS STREAM IMU ") + (imuStreamEnabled ? "ON" : "OFF"));
  emitLine(String("TEL STATUS SERVICE ") + (serviceMode ? "ON" : "OFF"));
  emitLine(String("TEL STATUS CALIBRATION ") + (elrsCalibrationActive ? "ACTIVE" : "IDLE") +
    " ARMED " + String(elrsCalibrationArmedOnce ? 1 : 0) +
    " TEST " + String(elrsCalibrationTestActive ? 1 : 0) +
    " EXPECTED " + String(elrsCalibrationExpectedIndex) + " " + String(elrsCalibrationExpectedPower));

  const float measuredBatteryVolts = readBatterySystemVoltage();
  const bool batteryValid = updateLatestBatteryVoltage(measuredBatteryVolts);
  const float batteryVolts = batteryValid ? measuredBatteryVolts :
    (latestBatteryVoltageValid ? latestBatteryVolts : 0.0f);
  float currentAmps = readCurrentSensorAmps();
  int batteryPct = batteryValid || latestBatteryVoltageValid ? estimateBatteryPercent(batteryVolts) : 0;

  emitLine("TEL STATUS BATTERY " + String(batteryVolts, 2) + " " + String(currentAmps, 2) + " " + String(batteryPct));
  emitLine("TEL STATUS CURRENT " + String(currentAmps, 2));

  emitWaterTemperatureTelemetry();
  emitWaterChemistryTelemetry();
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
  emitMagnetometerTelemetry();
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
  const bool missionCollectionSampling = missionExecutionActive && missionCheckpointActive;
  if (!ds18b20Available) {
    uint32_t retryIntervalMs = audioRecordingEnabled && !missionCollectionSampling ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : 5000UL;
    if (nowMs - lastDs18b20RequestMs >= retryIntervalMs) ds18b20Available = setupDs18b20();
    return;
  }

  uint32_t intervalMs = missionCollectionSampling ? SD_SCIENCE_INTERVAL_MS
    : (audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : WATER_TEMP_STREAM_INTERVAL_MS);
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

void emitMagnetometerTelemetry() {
  MagnetometerReadings readings = {};
  if (readQmc5883(readings)) {
    emitLine("TEL SCI MAG_RAW " + String(readings.x) + " " + String(readings.y) + " " + String(readings.z));
    if (magnetometerCalibrationLoaded) {
      const float calibratedX = (static_cast<float>(readings.x) - magnetometerOffsetX) * magnetometerScaleX;
      const float calibratedY = (static_cast<float>(readings.y) - magnetometerOffsetY) * magnetometerScaleY;
      float headingDegrees = atan2f(calibratedY, calibratedX) * 180.0f / PI;
      if (headingDegrees < 0.0f) headingDegrees += 360.0f;
      emitLine("TEL SCI MAG_HEADING " + String(headingDegrees, 1));
    }
  } else {
    emitLine("TEL SCI MAG_RAW UNKNOWN UNKNOWN UNKNOWN");
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

void readSessionManifestMetadata(const String& sessionPath, String& origin, String& missionId, String& missionRunId) {
  File manifest = SD.open((sessionPath + "/manifest.txt").c_str(), FILE_READ);
  if (!manifest) return;
  while (manifest.available()) {
    String line = manifest.readStringUntil('\n');
    line.trim();
    if (line.startsWith("session_origin=")) origin = line.substring(14);
    else if (line.startsWith("mission_id=")) missionId = line.substring(11);
    else if (line.startsWith("mission_run_id=")) missionRunId = line.substring(15);
    if (origin.length() && missionId.length() && missionRunId.length()) break;
  }
  manifest.close();
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
      if (directoryPath == "/" && entryPath.startsWith("/session")) {
        String origin, missionId, missionRunId;
        readSessionManifestMetadata(entryPath, origin, missionId, missionRunId);
        sdFileClient.print("|");
        sdFileClient.print(origin.length() ? origin : "unknown");
        sdFileClient.print("|");
        sdFileClient.print(missionId.length() ? missionId : "NONE");
        sdFileClient.print("|");
        sdFileClient.print(missionRunId.length() ? missionRunId : "NONE");
      }
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
  if (elrsCalibrationActive) stopElrsCalibration("SAFETY_STOP", false);
  bool wasActive = motorCommandActive();
  stopAllMotors();
  manualReturnHomeActive = false;
  controlMode = CONTROL_MODE_IDLE;
  controlSource = CONTROL_SOURCE_NONE;
  elrsArmed = false;
  elrsArmReady = false;
  holdPositionEnabled = false;
  if (wasActive) {
    recordLogLine("SAFETY MOTOR STOP: " + String(reason));
  }
}

void stopElrsForFailsafe(uint32_t now) {
  const bool rcTimedOut = !elrs.rcFresh(now);
  const bool linkFresh = elrs.linkFresh(now);
  const bool zeroLinkQuality = linkFresh && elrs.linkQuality() == 0;
  const char* reason = rcTimedOut && zeroLinkQuality ? "RC_TIMEOUT_AND_ZERO_LQ" :
                       rcTimedOut ? "RC_TIMEOUT" :
                       zeroLinkQuality ? "ZERO_LQ" : "ARM_STATE_LOST";
  const String rcAge = elrs.lastRcMs() == 0 ? String("UNKNOWN") : String(now - elrs.lastRcMs());
  const String linkAge = elrs.lastLinkMs() == 0 ? String("UNKNOWN") : String(now - elrs.lastLinkMs());
  const bool missionReturnGpsReady = gps.location.isValid() && gps.location.age() <= 1500 &&
    gps.satellites.isValid() && gps.satellites.value() >= 5 &&
    gps.hdop.isValid() && gps.hdop.hdop() <= 3.0;
  if (missionExecutionActive && missionMetadata.linkLossAction == 1 &&
      missionMetadata.hasHome && missionReturnGpsReady) {
    beginMissionReturnHome("ELRS LINK LOST");
    controlSource = CONTROL_SOURCE_NONE;
    elrsArmed = false;
    elrsArmReady = false;
  } else {
    if (missionExecutionActive) {
      stopMissionExecution(missionReturnGpsReady ? "ELRS LINK LOST" : "ELRS LINK LOST GPS UNAVAILABLE", false);
    }
    stopMotorsForSafety(F("ELRS FAILSAFE"));
  }
  manualLinkLossStop = true;
  ++elrsFailsafeCount;
  lastElrsFailsafeStatus = String("COUNT ") + String(elrsFailsafeCount) +
    " REASON " + reason +
    " RC_AGE_MS " + rcAge +
    " LQ " + String(elrs.linkQuality()) +
    " LINK_AGE_MS " + linkAge +
    " RC " + String(elrs.validRcFrames()) +
    " RADIO " + String(elrs.validRadioStatus()) +
    " CRCERR " + String(elrs.crcErrors()) +
    " UPTIME_MS " + String(now);
  recordLogLine(String("ELRS FAILSAFE ") + lastElrsFailsafeStatus);
  emitLine(String("TEL EVENT ELRS_FAILSAFE ") + lastElrsFailsafeStatus);
  if (!missionExecutionActive) {
    stopRecordingForManualLinkLoss();
  }
}

void updateMotorCommandWatchdog() {
  if (missionExecutionActive) return;
  if (controlSource == CONTROL_SOURCE_ELRS) {
    const uint32_t now = millis();
    if (!elrsArmed || !elrs.rcFresh(now) || (elrs.linkFresh(now) && elrs.linkQuality() == 0)) {
      stopElrsForFailsafe(now);
    }
    return;
  }
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

void updateMotorStallProtection(uint32_t now) {
  if (!motorCommandActive()) {
    motorStallNoMovementSinceMs = 0;
    motorStallGpsAnchorValid = false;
    return;
  }
  if (lastMotorStallSampleMs != 0 && now - lastMotorStallSampleMs < MOTOR_STALL_SAMPLE_INTERVAL_MS) return;
  lastMotorStallSampleMs = now;

  const float currentVoltage = readCurrentSensorVoltage();
  const float currentAmps = updateLatestCurrentSample(currentVoltage);
  const bool gpsFresh = gps.location.isValid() && gps.location.age() <= 1500;
  if (!isfinite(currentAmps) || currentAmps <= MOTOR_STALL_OVERCURRENT_AMPS || !gpsFresh) {
    motorStallNoMovementSinceMs = 0;
    motorStallGpsAnchorValid = false;
    return;
  }

  const double latitude = gps.location.lat();
  const double longitude = gps.location.lng();
  if (!motorStallGpsAnchorValid || motorStallNoMovementSinceMs == 0) {
    motorStallGpsAnchorLatitude = latitude;
    motorStallGpsAnchorLongitude = longitude;
    motorStallGpsAnchorValid = true;
    motorStallNoMovementSinceMs = now;
    return;
  }

  const double movedMeters = TinyGPSPlus::distanceBetween(latitude, longitude,
    motorStallGpsAnchorLatitude, motorStallGpsAnchorLongitude);
  if (!isfinite(movedMeters)) {
    motorStallNoMovementSinceMs = 0;
    motorStallGpsAnchorValid = false;
    return;
  }
  if (movedMeters >= MOTOR_STALL_GPS_MOVEMENT_METERS) {
    motorStallGpsAnchorLatitude = latitude;
    motorStallGpsAnchorLongitude = longitude;
    motorStallNoMovementSinceMs = now;
    return;
  }
  if (now - motorStallNoMovementSinceMs < MOTOR_STALL_OVERCURRENT_MS) return;

  recordLogLine(String("MOTOR STALL PROTECTION current_a=") + String(currentAmps, 2) +
    " duration_ms=" + String(now - motorStallNoMovementSinceMs) + " gps_moved_m=" + String(movedMeters, 2));
  emitLine(String("TEL EVENT MOTOR_STALL_OVERCURRENT ") + String(currentAmps, 2) + "A");
  if (missionExecutionActive) stopMissionExecution("MOTOR_STALL_OVERCURRENT", false);
  else stopMotorsForSafety(F("MOTOR STALL OVERCURRENT"));
  motorStallNoMovementSinceMs = 0;
  motorStallGpsAnchorValid = false;
}

void processTcpControl() {
  if (WiFi.status() != WL_CONNECTED) {
    if (controlSource == CONTROL_SOURCE_WIFI && (hadControlClient || motorCommandActive())) {
      stopMotorsForSafety(F("WI-FI LOST"));
    }
    hadControlClient = false;
    if (tcpClient) {
      tcpClient.stop();
    }
    tcpCommandBuffer = "";
    tcpTelemetryBuffered = 0;
    return;
  }

  if (hadControlClient && !controlClientConnected()) {
    serviceMode = false;
    audioStreamEnabled = false;
    imuStreamEnabled = false;
    if (controlSource == CONTROL_SOURCE_WIFI) stopMotorsForSafety(F("TCP CLIENT DISCONNECTED"));
    tcpClient.stop();
    tcpCommandBuffer = "";
    tcpTelemetryBuffered = 0;
    hadControlClient = false;
  }

  WiFiClient newClient = tcpServer.available();
  if (newClient) {
    serviceMode = false;
    audioStreamEnabled = false;
    imuStreamEnabled = false;
    if (tcpClient && tcpClient.connected()) {
      if (controlSource == CONTROL_SOURCE_WIFI) stopMotorsForSafety(F("CONTROL CLIENT REPLACED"));
      tcpClient.stop();
    }
    tcpClient = newClient;
    tcpTelemetryBuffered = 0;
    tcpClient.setNoDelay(true);
    udpRemoteIp = tcpClient.remoteIP();
    tcpCommandBuffer = "";
    hadControlClient = true;
    markControlHeartbeat();
    emitLine("ACK BOOT");
  }

  unsigned inputBytes = 0;
  unsigned inputCommands = 0;
  while (inputBytes < 256 && inputCommands < 2 &&
         controlClientConnected() && tcpClient.available() > 0) {
    ++inputBytes;
    char c = static_cast<char>(tcpClient.read());
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      ++inputCommands;
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

void computeMotorThrusts(float lateral, float thrust, float yaw, int* thrusts,
                         bool tx12ManualMix) {
  if (tx12ManualMix && mixerCalProfileLoaded && mixerCalibrationComplete()) {
    float calibrated[MOTOR_COUNT] = {0.0f, 0.0f, 0.0f};
    addMixerCalibrationAxis(thrust, 0, 1, calibrated);
    addMixerCalibrationAxis(lateral, 2, 3, calibrated);
    addMixerCalibrationAxis(yaw, 4, 5, calibrated);
    float maxMagnitude = 0.0f;
    for (int i = 0; i < MOTOR_COUNT; ++i) maxMagnitude = max(maxMagnitude, fabsf(calibrated[i]));
    const float scale = maxMagnitude > 100.0f ? 100.0f / maxMagnitude : 1.0f;
    for (int i = 0; i < MOTOR_COUNT; ++i)
      thrusts[i] = static_cast<int>(roundf(constrain(calibrated[i] * scale, -100.0f, 100.0f) * MOTOR_OUTPUT_LIMIT / 100.0f));
    return;
  }

  float mixedThrusts[3];
  float maxMagnitude = 0.0f;
  for (int i = 0; i < MOTOR_COUNT; i++) {
    const MotorChannel& motor = motors[i];
    const float axisX = USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT
      ? (tx12ManualMix
          ? (lateral > 0.0f ? tx12_manual_right_lateral_gains[i]
                            : tx12_manual_lateral_gains[i])
          : external_tangential_motor_axes[i][0])
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
    if (tx12ManualMix && USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT) {
      // The trial showed front-right+ turns toward negative yaw (left), and
      // front-left+ turns toward positive yaw (right).
      if (i == 2 && yaw < 0.0f) motorThrust -= yaw;
      if (i == 1 && yaw > 0.0f) motorThrust += yaw;
    } else {
      motorThrust += yaw * yawGain;
    }
    motorThrust += motor.mixBias;
    if (tx12ManualMix && USE_EXTERNAL_TANGENTIAL_MOTOR_LAYOUT && motorThrust < 0.0f) {
      motorThrust *= TX12_MANUAL_REVERSE_COMMAND_GAIN;
    }
    mixedThrusts[i] = motorThrust;
    maxMagnitude = max(maxMagnitude, abs(motorThrust));
  }

  // Scale the complete TX12 vector together so clipping cannot change its
  // motor proportions, including when the weak reverse motor sets the limit.
  float outputScale = ((tx12ManualMix || yaw != 0.0f) && maxMagnitude > 1.0f)
    ? (1.0f / maxMagnitude) : 1.0f;
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

  if (magnitude < LOW_SPEED_MIX_THRESHOLD) {
    const uint32_t onTimeMs = static_cast<uint32_t>(magnitude) *
      LOW_SPEED_MIX_PERIOD_MS / LOW_SPEED_MIX_THRESHOLD;
    const uint32_t phaseMs = (millis() - lowSpeedMixEpochMs) % LOW_SPEED_MIX_PERIOD_MS;
    if (phaseMs >= onTimeMs) return 0;
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
  if (constrainedDrive != 0) {
    bool allTargetsStopped = true;
    for (int i = 0; i < MOTOR_COUNT; i++) {
      if (targetDriveValues[i] != 0) {
        allTargetsStopped = false;
        break;
      }
    }
    if (allTargetsStopped) lowSpeedMixEpochMs = millis();
  }
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

void applyMotionCommand(int lateral, int thrust, int yaw, bool tx12ManualMix) {
  latestMixerInputLateral = lateral;
  latestMixerInputThrust = thrust;
  latestMixerInputYaw = yaw;
  latestMixerInputTx12 = tx12ManualMix;
  float lateralf = lateral / static_cast<float>(VECTOR_SCALE);
  float thrustf = thrust / static_cast<float>(VECTOR_SCALE);
  float yawf = yaw / static_cast<float>(VECTOR_SCALE);

  if (lateral == 0 && thrust == 0 && yaw == 0) {
    stopAllMotors();
    return;
  }

  int thrusts[3];
  computeMotorThrusts(lateralf, thrustf, yawf, thrusts, tx12ManualMix);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, thrusts[i]);
  }
}

void applyMotionCommand(int lateral, int thrust, int yaw) {
  applyMotionCommand(lateral, thrust, yaw, false);
}

void applyVectorCommand(int turn, int thrust) {
  applyMotionCommand(turn, thrust, 0);
}

int elrsStickPercent(uint16_t microseconds) {
  int offset = constrain(static_cast<int>(microseconds) - 1500, -500, 500);
  if (abs(offset) <= 40) return 0;
  return constrain(static_cast<int>(roundf(offset / 5.0f)), -100, 100);
}

String currentScienceTimestamp() {
  char timestamp[24];
  if (gps.date.isValid() && gps.time.isValid()) {
    snprintf(timestamp, sizeof(timestamp), "%04u%02u%02uT%02u%02u%02uZ",
      gps.date.year(), gps.date.month(), gps.date.day(),
      gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    // Keep the session folder's timestamp shape while marking UTC as unknown.
    const uint32_t elapsedSeconds = millis() / 1000UL;
    snprintf(timestamp, sizeof(timestamp), "00000000T%02lu%02lu%02luZ",
      static_cast<unsigned long>((elapsedSeconds / 3600UL) % 24UL),
      static_cast<unsigned long>((elapsedSeconds / 60UL) % 60UL),
      static_cast<unsigned long>(elapsedSeconds % 60UL));
  }
  return String(timestamp);
}

void processElrsScienceSwitch(bool manualMode) {
  const uint16_t channel = elrs.channel(7);  // SC / CH8
  if (channel < 900 || channel > 2100) return;

  // Low closes the session, middle records science without audio, and high
  // records science with audio. Only react to a switch edge.
  const int8_t position = channel < 1300 ? -1 : (channel >= 1700 ? 1 : 0);
  const bool positionWasKnown = elrsScienceSwitchPositionKnown;
  if (!positionWasKnown) {
    elrsScienceSwitchPosition = position;
    elrsScienceSwitchPositionKnown = true;
  }
  const bool changed = positionWasKnown && position != elrsScienceSwitchPosition;
  elrsScienceSwitchPosition = position;
  const bool enteringManual = manualMode && (!elrsScienceManualModeKnown || !elrsScienceManualMode);
  elrsScienceManualModeKnown = true;
  elrsScienceManualMode = manualMode;
  if (!manualMode || serviceMode || sdTransferActive) return;

  if (position < 0) {
    if (!changed) return;
    const String timestamp = currentScienceTimestamp();
    if (scienceExperimentState == 1) {
      handleControlCommand("CTRL SCIENCE STOP " + timestamp);
    }
    if (scienceExperimentState == 2) {
      handleControlCommand("CTRL SCIENCE SAVE " + timestamp);
    }
  } else if (position == 0) {
    if (scienceExperimentState == 0 && (changed || enteringManual || !positionWasKnown)) {
      handleControlCommand("CTRL SCIENCE START " + currentScienceTimestamp());
    }
    if (scienceExperimentState == 1 && audioRecordingEnabled) {
      handleControlCommand("CTRL AUDIO RECORD STOP");
    }
  } else if (position > 0) {
    if (scienceExperimentState == 0 && (changed || enteringManual || !positionWasKnown)) {
      handleControlCommand("CTRL SCIENCE START " + currentScienceTimestamp());
    }
    if (scienceExperimentState == 1) handleControlCommand("CTRL AUDIO RECORD START");
  }
}

const char* elrsCalibrationAxisName(uint8_t index) {
  static const char* names[MIXER_CAL_AXIS_COUNT] = {
    "forward", "reverse", "strafe_right", "strafe_left", "yaw_cw", "yaw_ccw"
  };
  return index < MIXER_CAL_AXIS_COUNT ? names[index] : "invalid";
}

bool startElrsCalibrationLog(uint8_t mode, uint8_t maxPower, uint32_t pulseDurationMs) {
  if (!sdCardMounted || mode > 1 || maxPower < 30 || maxPower > 100 || maxPower % 10 ||
      (pulseDurationMs != ELRS_CALIBRATION_PULSE_DURATION_DEFAULT_MS &&
       pulseDurationMs != ELRS_CALIBRATION_PULSE_DURATION_LONG_MS)) return false;
  elrsCalibrationPulseDurationMs = pulseDurationMs;
  if (!SD.exists("/calibration") && !SD.mkdir("/calibration")) return false;
  const String stamp = currentScienceTimestamp();
  char path[64];
  uint8_t collision = 0;
  do {
    snprintf(path, sizeof(path), "/calibration/%s_%s_%lu_%u.csv", mode == 0 ? "mixer" : "magmotor",
      stamp.c_str(), static_cast<unsigned long>(millis()), collision++);
  } while (SD.exists(path) && collision < 20);
  if (SD.exists(path)) return false;
  elrsCalibrationPath = path;
  sdCalibrationFile = SD.open(elrsCalibrationPath.c_str(), FILE_WRITE);
  if (!sdCalibrationFile) return false;

  sdCalibrationFile.println(F("#format=BUOY_ELRS_CALIBRATION_V1"));
  sdCalibrationFile.println(String("#boot_id=") + standaloneBootId());
  sdCalibrationFile.println(String("#started_utc=") + stamp);
  sdCalibrationFile.println(String("#firmware_version=") + firmwareVersionString());
  sdCalibrationFile.println(String("#test_mode=") + (mode == 0 ? "mixer_response" : "magnetometer_motor_load"));
  sdCalibrationFile.println(String("#max_test_power_percent=") + String(maxPower));
  sdCalibrationFile.println(F("#control_mode=manual"));
  sdCalibrationFile.println(String("#test_duration_ms=") + String(elrsCalibrationPulseDurationMs));
  sdCalibrationFile.println(String("#sample_interval_ms=") + String(ELRS_CALIBRATION_SAMPLE_INTERVAL_MS));
  sdCalibrationFile.println(F("#calibration_priority_mode=1"));
  sdCalibrationFile.println(String("#motor_count=") + String(MOTOR_COUNT));
  sdCalibrationFile.println(String("#mixer_profile_loaded=") + String(mixerCalProfileLoaded ? 1 : 0));
  sdCalibrationFile.println(String("#mixer_profile_complete=") + String(mixerCalibrationComplete() ? 1 : 0));
  sdCalibrationFile.println(String("#mixer_profile_source=") + (mixerCalLoadedFromBackup ? "backup" : "primary_or_none"));
  bool profileCrcValid = false;
  const char* activeProfilePath = mixerCalLoadedFromBackup ? "/config/propulsion.bak" : "/config/propulsion.cfg";
  const uint32_t activeProfileCrc = mixerCalProfileLoaded ? missionCrc32File(activeProfilePath, profileCrcValid) : 0;
  sdCalibrationFile.println(String("#mixer_profile_crc32=") + (profileCrcValid ? String(activeProfileCrc, HEX) : String("NONE")));
  sdCalibrationFile.println(String("#elrs_motor_power_limit_percent=") + String(ELRS_MOTOR_POWER_LIMIT_PERCENT));
  sdCalibrationFile.println(String("#motor_output_limit=") + String(MOTOR_OUTPUT_LIMIT));
  sdCalibrationFile.println(String("#motor_pwm_frequency_hz=") + String(PWM_FREQ_HZ));
  sdCalibrationFile.println(String("#motor_pwm_resolution_bits=") + String(PWM_RES_BITS));
  sdCalibrationFile.println(String("#motor_output_boost=") + String(MOTOR_OUTPUT_BOOST, 4));
  sdCalibrationFile.println(String("#motor_ramp_interval_ms=") + String(MOTOR_RAMP_INTERVAL_MS));
  sdCalibrationFile.println(String("#low_speed_mix_threshold=") + String(LOW_SPEED_MIX_THRESHOLD));
  sdCalibrationFile.println(String("#low_speed_mix_period_ms=") + String(LOW_SPEED_MIX_PERIOD_MS));
  sdCalibrationFile.println(String("#manual_reverse_command_gain=") + String(TX12_MANUAL_REVERSE_COMMAND_GAIN, 4));
  sdCalibrationFile.println(String("#tx12_manual_left_lateral_gains=") + String(tx12_manual_lateral_gains[0], 5) + "," +
    String(tx12_manual_lateral_gains[1], 5) + "," + String(tx12_manual_lateral_gains[2], 5));
  sdCalibrationFile.println(String("#tx12_manual_right_lateral_gains=") + String(tx12_manual_right_lateral_gains[0], 5) + "," +
    String(tx12_manual_right_lateral_gains[1], 5) + "," + String(tx12_manual_right_lateral_gains[2], 5));
  sdCalibrationFile.println(String("#magnetometer_available=") + String(magnetometerAvailable ? 1 : 0));
  sdCalibrationFile.println(String("#magnetometer_calibration_loaded=") + String(magnetometerCalibrationLoaded ? 1 : 0));
  sdCalibrationFile.println(String("#magnetometer_offset_x=") + String(magnetometerOffsetX, 6));
  sdCalibrationFile.println(String("#magnetometer_offset_y=") + String(magnetometerOffsetY, 6));
  sdCalibrationFile.println(String("#magnetometer_scale_x=") + String(magnetometerScaleX, 6));
  sdCalibrationFile.println(String("#magnetometer_scale_y=") + String(magnetometerScaleY, 6));
  sdCalibrationFile.println(F("#radio_channels=CH1_lateral,CH2_thrust,CH3_power,CH4_yaw,CH5_arm,CH6_telemetry,CH7_mode,CH8_session,CH9_home,CH10_rth,CH11_cal_enable,CH12_cal_command,CH13_cal_mode,CH14_test_index,CH15_test_power,CH16_cal_duration"));
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    const MotorChannel& m = motors[i];
    sdCalibrationFile.printf("#motor_%d=%s,direction_inverted:%d,deadband:%d,boost_pwm_fwd:%d,boost_pwm_rev:%d,boost_ms_fwd:%d,boost_ms_rev:%d,min_pwm_fwd:%d,min_pwm_rev:%d,max_pwm_fwd:%d,max_pwm_rev:%d,curve_fwd:%.5f,curve_rev:%.5f,scale_fwd:%.5f,scale_rev:%.5f,offset_fwd:%d,offset_rev:%d,ramp_up:%d,ramp_down:%d,mix_turn:%.5f,mix_thrust:%.5f,mix_bias:%.5f\n",
      i, i == 0 ? "rear" : (i == 1 ? "front_left" : "front_right"), m.directionInverted ? 1 : 0,
      m.commandDeadband, m.startBoostPwmForward, m.startBoostPwmReverse, m.startBoostMsForward,
      m.startBoostMsReverse, m.minStartPwmForward, m.minStartPwmReverse, m.maxPwmForward,
      m.maxPwmReverse, m.pwmCurveForward, m.pwmCurveReverse, m.pwmScaleForward, m.pwmScaleReverse,
      m.pwmOffsetForward, m.pwmOffsetReverse, m.rampUpPerSecond, m.rampDownPerSecond,
      m.mixTurnGain, m.mixThrustGain, m.mixBias);
  }
  sdCalibrationFile.println(F("#profile_point_columns=power_percent,axis,rear_percent,front_left_percent,front_right_percent,set"));
  for (uint8_t step = 0; step < MIXER_CAL_POWER_STEPS; ++step) {
    for (uint8_t axis = 0; axis < MIXER_CAL_AXIS_COUNT; ++axis) {
      sdCalibrationFile.printf("#profile_point=%u,%s,%d,%d,%d,%d\n", (step + 1) * 10,
        elrsCalibrationAxisName(axis), mixerCalValues[step][axis][0], mixerCalValues[step][axis][1],
        mixerCalValues[step][axis][2], mixerCalPointSet[step][axis] ? 1 : 0);
    }
  }
  sdCalibrationFile.println(F("#data follows; all motor commands are signed drive values in [-255,255], PWM columns are applied output PWM"));
  sdCalibrationFile.println(F("record_type,elapsed_ms,test_index,test_name,requested_power_pct,test_active,commanded_lateral,commanded_thrust,commanded_yaw,drive_rear,drive_front_left,drive_front_right,target_rear,target_front_left,target_front_right,pwm_rear,pwm_front_left,pwm_front_right,current_a,battery_v,current_valid,battery_valid,accel_x_g,accel_y_g,accel_z_g,gyro_x_dps,gyro_y_dps,gyro_z_dps,mag_x_raw,mag_y_raw,mag_z_raw,mag_heading_deg,gps_valid,latitude,longitude,speed_mps,course_deg,satellites,hdop,ch1,ch2,ch3,ch4,ch5,ch7,ch11,ch12,ch13,ch14,ch15,mag_available,mag_valid,mag_status,mag_samples_ok,mag_not_ready,mag_overflows,mag_i2c_errors,gps_age_ms"));
  sdCalibrationFile.flush();
  if (sdCalibrationFile.getWriteError()) {
    sdCalibrationFile.close();
    return false;
  }
  elrsCalibrationMode = mode;
  elrsCalibrationPowerPercent = maxPower;
  elrsCalibrationExpectedIndex = 0;
  elrsCalibrationExpectedPower = 30;
  elrsCalibrationActive = true;
  elrsCalibrationStatusCode = 1;
  elrsCalibrationArmedOnce = false;
  elrsCalibrationArmBlockLogged = false;
  elrsCalibrationTestActive = false;
  elrsCalibrationFinalTest = false;
  elrsCalibrationSampleCount = 0;
  elrsCalibrationStartedMs = millis();
  elrsCalibrationCurrentPowerPercent = 0;
  elrsCalibrationSampleMs = millis();
  elrsCalibrationFlushMs = millis();
  elrs.sendScienceValue("CAL_STAT", 1);
  emitLine(String("TEL EVENT CALIBRATION STARTED ") + elrsCalibrationPath);
  recordLogLine(String("ELRS CALIBRATION STARTED ") + elrsCalibrationPath);
  return true;
}

void stopElrsCalibration(const char* reason, bool completed) {
  if (!elrsCalibrationActive) return;
  stopAllMotors();
  if (sdCalibrationFile) {
    sdCalibrationFile.printf("#end_elapsed_ms=%lu\n#stop_reason=%s\n#completed=%u\n#sample_count=%lu\n",
      static_cast<unsigned long>(millis() - elrsCalibrationStartedMs), reason ? reason : "UNKNOWN",
      completed ? 1 : 0, static_cast<unsigned long>(elrsCalibrationSampleCount));
    sdCalibrationFile.flush();
    sdCalibrationFile.close();
  }
  elrsCalibrationTestActive = false;
  elrsCalibrationFinalTest = false;
  elrsCalibrationActive = false;
  elrsCalibrationStatusCode = completed ? 3 : 2;
  elrsCalibrationLastCommand = 0;
  elrsArmed = false;
  elrsArmReady = false;
  controlSource = CONTROL_SOURCE_NONE;
  controlMode = CONTROL_MODE_IDLE;
  elrs.sendScienceValue("CAL_STAT", completed ? 3 : 2);
  emitLine(String("TEL EVENT CALIBRATION ") + (completed ? "COMPLETE " : "STOPPED ") + (reason ? reason : "UNKNOWN"));
  recordLogLine(String("ELRS CALIBRATION ") + (completed ? "COMPLETE " : "STOPPED ") + (reason ? reason : "UNKNOWN"));
}

void processElrsCalibrationCommand(uint32_t now) {
  const uint16_t marker = elrs.channel(10);
  const uint16_t command = elrs.channel(11);
  const bool enabled = marker >= 1700;
  const bool commandValid = command >= 900 && command <= 2100;
  uint8_t decodedCommand = 0;
  if (commandValid) {
    if (command >= 1100 && command <= 1300) decodedCommand = 1; // START
    else if (command >= 1400 && command <= 1600) decodedCommand = 2; // NEXT
    else if (command >= 1700) decodedCommand = 3; // STOP
  }
  const bool commandEdge = decodedCommand != 0 && decodedCommand != elrsCalibrationLastCommand;
  if (decodedCommand == 0) elrsCalibrationLastCommand = 0;

  const uint16_t modeChannel = elrs.channel(12);
  const uint16_t indexChannel = elrs.channel(13);
  const uint16_t requestedPowerChannel = elrs.channel(14);
  const uint16_t durationChannel = elrs.channel(15);
  if (!elrsCalibrationActive) {
    if (enabled && decodedCommand == 1 && commandEdge) {
      const bool manual = elrs.channel(6) >= 900 && elrs.channel(6) < 1300;
      const bool disarmed = elrs.channel(4) < 1300 && !elrsArmed;
      const bool centered = abs(elrsStickPercent(elrs.channel(0))) <= 2 &&
        abs(elrsStickPercent(elrs.channel(1))) <= 2 && abs(elrsStickPercent(elrs.channel(3))) <= 2;
      const bool powerMinimum = elrs.channel(2) >= 900 && elrs.channel(2) <= 1100;
      const bool commandFieldsValid = modeChannel >= ELRS_CALIBRATION_CHANNEL_MIN_US &&
        modeChannel <= ELRS_CALIBRATION_CHANNEL_MAX_US &&
        indexChannel >= ELRS_CALIBRATION_CHANNEL_MIN_US && indexChannel <= ELRS_CALIBRATION_CHANNEL_MAX_US &&
        requestedPowerChannel >= ELRS_CALIBRATION_CHANNEL_MIN_US &&
        requestedPowerChannel <= ELRS_CALIBRATION_CHANNEL_MAX_US &&
        durationChannel >= ELRS_CALIBRATION_CHANNEL_MIN_US &&
        durationChannel <= ELRS_CALIBRATION_CHANNEL_MAX_US;
      if (!manual || !disarmed || !centered || !powerMinimum || !sdCardMounted ||
          serviceMode || sdTransferActive || scienceExperimentState != 0 ||
          controlSource == CONTROL_SOURCE_WIFI || !commandFieldsValid) {
        elrs.sendScienceValue("CAL_STAT", 4);
        char details[320];
        snprintf(details, sizeof(details),
          "INTERLOCK MANUAL=%u DISARMED=%u CENTERED=%u POWER_MIN=%u SD_READY=%u SERVICE=%u SD_TRANSFER=%u SCIENCE_IDLE=%u WIFI_CONTROL=%u FIELDS_VALID=%u RC_FRESH=%u GPS_FIX=%u PRIMARY=%u,%u,%u,%u,%u,%u CH11=%u CH12=%u CH13=%u CH14=%u CH15=%u CH16=%u",
          manual ? 1U : 0U, disarmed ? 1U : 0U, centered ? 1U : 0U, powerMinimum ? 1U : 0U,
          sdCardMounted ? 1U : 0U, serviceMode ? 1U : 0U, sdTransferActive ? 1U : 0U,
          scienceExperimentState == 0 ? 1U : 0U, controlSource == CONTROL_SOURCE_WIFI ? 1U : 0U,
          commandFieldsValid ? 1U : 0U, elrs.rcFresh(now) ? 1U : 0U, gps.location.isValid() ? 1U : 0U,
          elrs.channel(0), elrs.channel(1), elrs.channel(2), elrs.channel(3), elrs.channel(4), elrs.channel(6),
          marker, command, modeChannel, indexChannel, requestedPowerChannel, durationChannel);
        const String diagnostic = String("CALIBRATION START BLOCKED ") + details;
        emitLine(String("TEL EVENT ") + diagnostic);
        recordLogLine(diagnostic);
      } else {
        const uint8_t mode = modeChannel >= 1700 ? 1 : 0;
        // Preserve a minimum that can produce measurable propulsion in water.
        int maxPower = constrain((static_cast<int>(requestedPowerChannel) - 1000) / 10, 0, 100);
        maxPower = constrain(((maxPower + 5) / 10) * 10, 30, 100);
        const uint32_t pulseDurationMs = durationChannel >= 1700
          ? ELRS_CALIBRATION_PULSE_DURATION_LONG_MS : ELRS_CALIBRATION_PULSE_DURATION_DEFAULT_MS;
        if (!startElrsCalibrationLog(mode, static_cast<uint8_t>(maxPower), pulseDurationMs)) {
          elrs.sendScienceValue("CAL_STAT", 5);
          emitLine("TEL EVENT CALIBRATION BLOCKED SD_WRITE");
        } else {
          // START was accepted only with CH5 low. Preserve that verified
          // disarmed state if SE is raised before another RC frame arrives.
          elrsArmReady = true;
          char accepted[192];
          snprintf(accepted, sizeof(accepted),
            "CALIBRATION START ACCEPTED MODE=%u MAX_POWER=%u PULSE_MS=%lu CH13=%u CH14=%u CH15=%u CH16=%u",
            mode, static_cast<unsigned>(maxPower), static_cast<unsigned long>(pulseDurationMs),
            modeChannel, indexChannel, requestedPowerChannel, durationChannel);
          emitLine(String("TEL EVENT ") + accepted);
          recordLogLine(accepted);
        }
      }
    }
    elrsCalibrationLastCommand = decodedCommand;
    return;
  }

  if (!enabled) {
    stopElrsCalibration("TX12_COMMAND_ENABLE_LOST", false);
    return;
  }
  const uint32_t requestedDurationMs = durationChannel >= 1700
    ? ELRS_CALIBRATION_PULSE_DURATION_LONG_MS : ELRS_CALIBRATION_PULSE_DURATION_DEFAULT_MS;
  if (durationChannel < ELRS_CALIBRATION_CHANNEL_MIN_US ||
      durationChannel > ELRS_CALIBRATION_CHANNEL_MAX_US ||
      requestedDurationMs != elrsCalibrationPulseDurationMs) {
    stopElrsCalibration("DURATION_SWITCH_CHANGED", false);
    elrsArmed = false;
    controlSource = CONTROL_SOURCE_NONE;
    controlMode = CONTROL_MODE_IDLE;
    return;
  }
  if (decodedCommand == 3 && commandEdge) {
    stopElrsCalibration("TX12_STOP", false);
    return;
  }
  const uint16_t armChannel = elrs.channel(4);
  if (armChannel < 1300) {
    if (elrsCalibrationTestActive || elrsCalibrationArmedOnce) {
      stopElrsCalibration("DISARMED", false);
      elrsArmed = false;
      controlSource = CONTROL_SOURCE_NONE;
      controlMode = CONTROL_MODE_IDLE;
      return;
    }
    elrsArmReady = true;
    elrsArmed = false;
    controlSource = CONTROL_SOURCE_NONE;
    elrsCalibrationLastCommand = decodedCommand;
    return;
  }
  const bool manual = elrs.channel(6) >= 900 && elrs.channel(6) < 1300;
  const bool centered = abs(elrsStickPercent(elrs.channel(0))) <= 2 &&
    abs(elrsStickPercent(elrs.channel(1))) <= 2 && abs(elrsStickPercent(elrs.channel(3))) <= 2;
  // The low-SE arm latch is consumed on the first armed frame. Once this
  // calibration run has armed, keep accepting subsequent frames until SE is
  // lowered or another safety interlock fails.
  const bool calibrationArmAuthorized = elrsArmReady || elrsCalibrationArmedOnce;
  if (!manual || armChannel < 1700 || !calibrationArmAuthorized || !centered || elrs.channel(2) > 1100 ||
      serviceMode || sdTransferActive || missionExecutionActive || controlSource == CONTROL_SOURCE_WIFI) {
    if (!elrsCalibrationArmBlockLogged) {
      char blocked[190];
      snprintf(blocked, sizeof(blocked),
        "CALIBRATION ARM BLOCKED MANUAL=%u CH5=%u READY=%u ARMED_ONCE=%u CENTERED=%u CH3=%u SERVICE=%u SD_TRANSFER=%u MISSION=%u WIFI_CONTROL=%u",
        manual ? 1U : 0U, armChannel, elrsArmReady ? 1U : 0U,
        elrsCalibrationArmedOnce ? 1U : 0U, centered ? 1U : 0U, elrs.channel(2),
        serviceMode ? 1U : 0U, sdTransferActive ? 1U : 0U,
        missionExecutionActive ? 1U : 0U, controlSource == CONTROL_SOURCE_WIFI ? 1U : 0U);
      emitLine(String("TEL EVENT ") + blocked);
      recordLogLine(blocked);
      elrsCalibrationArmBlockLogged = true;
    }
    if (elrsArmed || elrsCalibrationTestActive) stopElrsCalibration("SAFETY_INTERLOCK", false);
    else elrs.sendScienceValue("CAL_STAT", 4);
    elrsArmed = false;
    controlSource = CONTROL_SOURCE_NONE;
    controlMode = CONTROL_MODE_IDLE;
    return;
  }
  if (!elrsCalibrationArmedOnce) {
    emitLine("TEL EVENT CALIBRATION ARMED");
    recordLogLine("ELRS CALIBRATION ARMED");
  }
  elrsArmed = true;
  elrsArmReady = false;
  elrsCalibrationArmedOnce = true;
  controlSource = CONTROL_SOURCE_ELRS;
  controlMode = CONTROL_MODE_MANUAL;
  if (commandEdge && decodedCommand == 2 && !elrsCalibrationTestActive) {
    const bool modeMatches = elrsCalibrationMode == 1
      ? modeChannel >= 1900 && modeChannel <= ELRS_CALIBRATION_CHANNEL_MAX_US
      : modeChannel >= ELRS_CALIBRATION_CHANNEL_MIN_US && modeChannel <= 1100;
    if (!modeMatches || indexChannel < ELRS_CALIBRATION_CHANNEL_MIN_US ||
        indexChannel > ELRS_CALIBRATION_CHANNEL_MAX_US ||
        requestedPowerChannel < ELRS_CALIBRATION_CHANNEL_MIN_US ||
        requestedPowerChannel > ELRS_CALIBRATION_CHANNEL_MAX_US) {
      char rejected[160];
      snprintf(rejected, sizeof(rejected),
        "CALIBRATION STEP REJECTED MODE=%u CH13=%u CH14=%u CH15=%u EXPECTED_INDEX=%u EXPECTED_POWER=%u",
        static_cast<unsigned>(elrsCalibrationMode), modeChannel, indexChannel,
        requestedPowerChannel, static_cast<unsigned>(elrsCalibrationExpectedIndex),
        static_cast<unsigned>(elrsCalibrationExpectedPower));
      emitLine(String("TEL EVENT ") + rejected);
      recordLogLine(rejected);
      stopElrsCalibration("INVALID_STEP_ARGUMENTS", false);
      elrsArmed = false;
      controlSource = CONTROL_SOURCE_NONE;
      controlMode = CONTROL_MODE_IDLE;
      return;
    }
    int testIndex = constrain((static_cast<int>(indexChannel) - 1000 + 100) / 200, 0, 5);
    int power = constrain((static_cast<int>(requestedPowerChannel) - 1000) / 10, 0, 100);
    power = constrain(((power + 5) / 10) * 10, 30, elrsCalibrationPowerPercent);
    if (testIndex != elrsCalibrationExpectedIndex || power != elrsCalibrationExpectedPower) {
      char mismatch[160];
      snprintf(mismatch, sizeof(mismatch),
        "CALIBRATION STEP MISMATCH INDEX=%u POWER=%u EXPECTED_INDEX=%u EXPECTED_POWER=%u CH12=%u CH14=%u CH15=%u",
        static_cast<unsigned>(testIndex), static_cast<unsigned>(power),
        static_cast<unsigned>(elrsCalibrationExpectedIndex),
        static_cast<unsigned>(elrsCalibrationExpectedPower), command, indexChannel,
        requestedPowerChannel);
      emitLine(String("TEL EVENT ") + mismatch);
      recordLogLine(mismatch);
      stopElrsCalibration("TEST_SEQUENCE_MISMATCH", false);
      elrsArmed = false;
      controlSource = CONTROL_SOURCE_NONE;
      controlMode = CONTROL_MODE_IDLE;
      return;
    }
    if (elrsCalibrationMode == 0) {
      const int demand = power;
      int lateral = 0, thrust = 0, yaw = 0;
      if (testIndex == 0) thrust = demand;
      else if (testIndex == 1) thrust = -demand;
      else if (testIndex == 2) lateral = demand;
      else if (testIndex == 3) lateral = -demand;
      else if (testIndex == 4) yaw = demand;
      else yaw = -demand;
      applyMotionCommand(lateral, thrust, yaw, true);
      latestMixerInputLateral = lateral;
      latestMixerInputThrust = thrust;
      latestMixerInputYaw = yaw;
      latestMixerInputTx12 = true;
    } else {
      const int motorIndex = testIndex / 2;
      const bool forward = (testIndex % 2) == 0;
      const int drive = static_cast<int>(roundf(power * MOTOR_OUTPUT_LIMIT *
        (ELRS_MOTOR_POWER_LIMIT_PERCENT / 100.0f) / 100.0f));
      for (int i = 0; i < MOTOR_COUNT; ++i) setMotorDrive(i, i == motorIndex ? (forward ? drive : -drive) : 0);
      latestMixerInputLateral = 0;
      latestMixerInputThrust = 0;
      latestMixerInputYaw = 0;
      latestMixerInputTx12 = false;
    }
    elrsCalibrationTestIndex = static_cast<uint8_t>(testIndex);
    elrsCalibrationCurrentPowerPercent = static_cast<uint8_t>(power);
    elrsCalibrationTestActive = true;
    elrsCalibrationFinalTest = (power == elrsCalibrationPowerPercent && testIndex == 5);
    if (testIndex == 5) {
      if (power < elrsCalibrationPowerPercent) {
        elrsCalibrationExpectedIndex = 0;
        elrsCalibrationExpectedPower = static_cast<uint8_t>(power + 10);
      }
    } else {
      elrsCalibrationExpectedIndex = static_cast<uint8_t>(testIndex + 1);
      elrsCalibrationExpectedPower = static_cast<uint8_t>(power);
    }
    elrsCalibrationTestStartedMs = now;
    elrsCalibrationTestEndsMs = now + elrsCalibrationPulseDurationMs;
    elrs.sendScienceValue("CAL_STEP", testIndex + 1);
    elrs.sendScienceValue("CAL_PWR", power);
    emitLine(String("TEL EVENT CALIBRATION STEP ") + String(testIndex) + " " + String(power));
  }
  elrsCalibrationLastCommand = decodedCommand;
}

void serviceElrsCalibration(uint32_t now) {
  if (!elrsCalibrationActive) return;
  if (serviceMode || sdTransferActive || missionExecutionActive || scienceExperimentState != 0 ||
      controlSource == CONTROL_SOURCE_WIFI) {
    stopElrsCalibration("EXTERNAL_CONTROL_OR_SD_ACTIVITY", false);
    return;
  }
  if (!elrs.rcFresh(now) || (elrs.linkFresh(now) && elrs.linkQuality() == 0)) {
    stopElrsCalibration("ELRS_FAILSAFE", false);
    return;
  }
  if (elrsCalibrationTestActive && static_cast<int32_t>(now - elrsCalibrationTestEndsMs) >= 0) {
    stopAllMotors();
    elrsCalibrationTestActive = false;
    if (elrsCalibrationFinalTest) {
      stopElrsCalibration("ALL_STEPS_COMPLETE", true);
      return;
    }
    elrs.sendScienceValue("CAL_STAT", 1);
  }
  if (now - elrsCalibrationSampleMs >= ELRS_CALIBRATION_SAMPLE_INTERVAL_MS) {
    elrsCalibrationSampleMs = now;
    writeElrsCalibrationSample(now);
  }
  if (now - elrsCalibrationFlushMs >= 1000) {
    elrsCalibrationFlushMs = now;
    sdCalibrationFile.flush();
    if (sdCalibrationFile.getWriteError()) stopElrsCalibration("SD_WRITE_ERROR", false);
  }
}

void writeElrsCalibrationSample(uint32_t now) {
  if (!sdCalibrationFile) return;
  ImuReadings imu = {};
  MagnetometerReadings mag = {};
  const bool imuValid = readMpu6050(imu);
  const bool magValid = readQmc5883(mag);
  const float currentSensorVoltage = readCurrentSensorVoltage();
  const bool currentValid = isfinite(currentSensorVoltage);
  const float sampleCurrent = currentValid ? currentAmpsFromSensorVoltage(currentSensorVoltage) : NAN;
  if (currentValid) updateLatestCurrentSample(currentSensorVoltage);
  const float sampleBattery = readBatterySystemVoltage();
  const bool batteryValid = updateLatestBatteryVoltage(sampleBattery);
  float heading = NAN;
  if (magValid && imuValid && magnetometerCalibrationLoaded) {
    calculateTiltHeadingDegrees(mag, imu, heading);
  }
  const bool gpsValid = gps.location.isValid() && gps.location.age() < 2000;
  const int lateral = latestMixerInputLateral;
  const int thrust = latestMixerInputThrust;
  const int yaw = latestMixerInputYaw;
  const char* testName = elrsCalibrationMode == 0 ? elrsCalibrationAxisName(elrsCalibrationTestIndex) :
    (elrsCalibrationTestIndex == 0 ? "rear_forward" : elrsCalibrationTestIndex == 1 ? "rear_reverse" :
     elrsCalibrationTestIndex == 2 ? "front_left_forward" : elrsCalibrationTestIndex == 3 ? "front_left_reverse" :
     elrsCalibrationTestIndex == 4 ? "front_right_forward" : "front_right_reverse");
  sdCalibrationFile.printf("SAMPLE,%lu,%u,%s,%u,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%u,%u,",
    static_cast<unsigned long>(now - elrsCalibrationStartedMs), static_cast<unsigned int>(elrsCalibrationTestIndex),
    testName, static_cast<unsigned int>(elrsCalibrationCurrentPowerPercent),
    elrsCalibrationTestActive ? 1U : 0U,
    lateral, thrust, yaw, driveValues[0], driveValues[1], driveValues[2], targetDriveValues[0], targetDriveValues[1],
    targetDriveValues[2], appliedMotorPwm[0], appliedMotorPwm[1], appliedMotorPwm[2], sampleCurrent,
    batteryValid ? sampleBattery : NAN, currentValid ? 1U : 0U, batteryValid ? 1U : 0U);
  if (imuValid) sdCalibrationFile.printf("%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,", imu.accelXG, imu.accelYG, imu.accelZG, imu.gyroXDps, imu.gyroYDps, imu.gyroZDps);
  else sdCalibrationFile.print(F("nan,nan,nan,nan,nan,nan,"));
  sdCalibrationFile.printf("%d,%d,%d,%.3f,%u,%.7f,%.7f,%.3f,%.3f,%u,%.2f,",
    magValid ? mag.x : 0, magValid ? mag.y : 0, magValid ? mag.z : 0, heading,
    gpsValid ? 1U : 0U, gpsValid ? gps.location.lat() : 0.0, gpsValid ? gps.location.lng() : 0.0,
    gps.speed.isValid() ? gps.speed.mps() : 0.0, gps.course.isValid() ? gps.course.deg() : 0.0,
    gps.satellites.isValid() ? static_cast<unsigned int>(gps.satellites.value()) : 0U,
    gps.hdop.isValid() ? gps.hdop.hdop() : 0.0);
  for (uint8_t channel = 0; channel < 5; ++channel) {
    sdCalibrationFile.printf("%u,", elrs.channel(channel));
  }
  sdCalibrationFile.printf("%u,%u,%u,%u,%u,%u,", elrs.channel(6), elrs.channel(10), elrs.channel(11),
    elrs.channel(12), elrs.channel(13), elrs.channel(14));
  sdCalibrationFile.printf("%u,%u,%u,%lu,%lu,%lu,%lu,", magnetometerAvailable ? 1U : 0U,
    magValid ? 1U : 0U, static_cast<unsigned int>(lastMagnetometerStatus),
    static_cast<unsigned long>(magnetometerSamplesOk), static_cast<unsigned long>(magnetometerNotReady),
    static_cast<unsigned long>(magnetometerOverflows), static_cast<unsigned long>(magnetometerI2cErrors));
  if (gps.location.isValid()) sdCalibrationFile.print(gps.location.age());
  sdCalibrationFile.println();
  ++elrsCalibrationSampleCount;
}

void processElrsControl() {
  elrs.poll();
  const uint32_t now = millis();
  if (elrs.rcFresh(now)) {
    const uint16_t telemetryChannel = elrs.channel(5);  // SF / CH6
    if (telemetryChannel >= 900 && telemetryChannel <= 2100) {
      const TelemetryMode requestedMode = telemetryChannel >= 1700
        ? TELEMETRY_MODE_MAVLINK : TELEMETRY_MODE_WIFI;
      if (requestedMode != telemetryMode) {
        telemetryMode = requestedMode;
        emitLine(String("TEL EVENT TELEMETRY_MODE ") +
          (telemetryMode == TELEMETRY_MODE_MAVLINK ? "MAVLINK" : "WIFI"));
      }
    }
  }
  if (!elrs.rcFresh(now) || (elrs.linkFresh(now) && elrs.linkQuality() == 0)) {
    elrsScienceSwitchPositionKnown = false;
    elrsScienceManualModeKnown = false;
    if (controlSource == CONTROL_SOURCE_ELRS && !(missionExecutionActive && missionReturningHome)) {
      stopElrsForFailsafe(now);
    }
    return;
  }

  // CH7 is the TX12 SB mode selector.
  const uint16_t missionModeChannel = elrs.channel(6);
  const bool manualModeSelected = missionModeChannel >= 900 && missionModeChannel < 1700;
  const bool requestedManualAssist = missionModeChannel >= 1300 && missionModeChannel < 1700;
  if (requestedManualAssist != manualAssistRequested) {
    manualAssistRequested = requestedManualAssist;
    manualAssistTargetValid = false;
    manualAssistYawing = false;
    manualAssistHeadingAvailable = false;
    manualAssistLastYawCorrection = 0.0f;
    lastManualAssistUpdateMs = 0;
    emitLine(String("TEL EVENT MANUAL_ASSIST ") + (manualAssistRequested ? "ON" : "OFF"));
  }
  processElrsCalibrationCommand(now);
  if (elrsCalibrationActive) return;
  processElrsScienceSwitch(manualModeSelected);
  if (missionModeChannel >= 1700) {
    if (!missionModeRequested) {
      if (elrsArmed && controlSource == CONTROL_SOURCE_ELRS) {
        stopMotorsForSafety(F("MISSION MODE SWITCH"));
      }
      emitLine("TEL EVENT MISSION_MODE REQUESTED");
    }
    missionModeRequested = true;
    const uint16_t armChannel = elrs.channel(4);  // CH5: low disarm, high arm
    if (missionStartLocked) {
      if (armChannel < 1300) {
        missionStartLocked = false;
        missionStartBlockCode = 0;
        missionStartBlockedLogged = false;
        elrsArmReady = true;
        elrs.sendScienceValue("MIS_BLOCK", 0);
        emitLine("TEL EVENT MISSION_RESTART_UNLOCKED ARM_SWITCH_LOW");
        recordLogLine("MISSION RESTART UNLOCKED: ARM SWITCH LOW");
      } else {
        reportMissionStartBlock(9, "CYCLE_ARM_LOW_TO_RESTART");
        return;
      }
    }
    if (armChannel < 1300) {
      if (missionExecutionActive) stopMissionExecution("RADIO DISARM", false);
      elrsArmReady = true;
      if (elrsArmed) stopMotorsForSafety(F("MISSION DISARMED"));
      elrsArmed = false;
      return;
    }
    if (armChannel < 1700 || serviceMode || sdTransferActive) {
      if (armChannel >= 1300) reportMissionStartBlock(14, "ARM_SWITCH_NOT_FULLY_HIGH");
      else if (serviceMode || sdTransferActive) reportMissionStartBlock(15, "BUOY_SERVICE_OR_SD_TRANSFER_ACTIVE");
      if (missionExecutionActive) stopMissionExecution("RADIO ARM SWITCH", false);
      if (elrsArmed) stopMotorsForSafety(F("MISSION DISARMED"));
      elrsArmed = false;
      return;
    }
    const int lateral = elrsStickPercent(elrs.channel(0));
    const int thrust = elrsStickPercent(elrs.channel(1));
    const int yaw = elrsStickPercent(elrs.channel(3));
    const uint16_t powerChannel = elrs.channel(2);
    if (!elrsArmed) {
      if (controlSource == CONTROL_SOURCE_WIFI) {
        reportMissionStartBlock(3, "WIFI_CONTROL_SOURCE_ACTIVE");
        if (!elrsArmBlockedByWifiLogged) recordLogLine("MISSION ARM BLOCKED: WIFI CONTROL SOURCE ACTIVE");
        elrsArmBlockedByWifiLogged = true;
        return;
      }
      if (!elrsArmReady || powerChannel > 1100 || lateral || thrust || yaw) {
        if (!elrsArmInterlockBlockLogged) {
          recordLogLine("MISSION ARM BLOCKED: CYCLE ARM LOW; POWER MINIMUM; STICKS CENTERED");
          elrsArmInterlockBlockLogged = true;
        }
        reportMissionStartBlock(!elrsArmReady ? 1 : 2,
          !elrsArmReady ? "CYCLE_ARM_LOW" : "POWER_MINIMUM_STICKS_CENTERED");
        return;
      }
      if (!missionId[0] || missionWaypointCount == 0 || !missionMetadata.hasHome || missionMetadata.boundaryCount < 3) {
        reportMissionStartBlock(10, "V4_MISSION_WITH_HOME_BOUNDARY_REQUIRED");
        missionStartBlockedLogged = true;
        return;
      }
      bool routeValid = missionCoordinateInsideBoundary(missionMetadata.home.latitude, missionMetadata.home.longitude);
      for (uint8_t i = 0; routeValid && i < missionWaypointCount; ++i) {
        routeValid = missionWaypoints[i].speedPercent >= 1 && missionWaypoints[i].speedPercent <= 100 &&
          missionCoordinateInsideBoundary(missionWaypoints[i].latitude, missionWaypoints[i].longitude);
      }
      if (!routeValid) {
        reportMissionStartBlock(13, "MISSION_ROUTE_POINT_OR_SPEED_INVALID");
        missionStartBlockedLogged = true;
        return;
      }
      if (!sdCardMounted) {
        reportMissionStartBlock(11, "SD_CARD_UNAVAILABLE");
        missionStartBlockedLogged = true;
        return;
      }
      if (scienceExperimentState != 0) {
        reportMissionStartBlock(12, "SAVE_OR_STOP_EXISTING_SESSION_FIRST");
        missionStartBlockedLogged = true;
        return;
      }
      if (!gps.location.isValid() || gps.location.age() > 1500 || !gps.satellites.isValid() ||
          gps.satellites.value() < 5 || !gps.hdop.isValid() || gps.hdop.hdop() > 3.0) {
        reportMissionStartBlock(5, "GPS_QUALITY_NEEDS_5_SATS_HDOP_3_OR_BETTER");
        missionStartBlockedLogged = true;
        return;
      }
      if (!missionCoordinateInsideBoundary(gps.location.lat(), gps.location.lng())) {
        reportMissionStartBlock(6, "GPS_OUTSIDE_MISSION_BOUNDARY");
        missionStartBlockedLogged = true;
        return;
      }
      if (TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),
          missionMetadata.home.latitude, missionMetadata.home.longitude) > MISSION_START_RADIUS_M) {
        reportMissionStartBlock(7, "MOVE_WITHIN_30M_OF_MISSION_HOME");
        missionStartBlockedLogged = true;
        return;
      }
      // Holding SA while switching to Mission deliberately captures this
      // start position as Home for the run. The original mission file remains
      // unchanged; the session manifest and event log record the run Home.
      const bool captureMissionHome = elrs.channel(8) >= 1500;
      const auto plannedMissionHome = missionMetadata.home;
      if (captureMissionHome && missionWaypointCount > 0 &&
          !missionSegmentInsideBoundary(gps.location.lat(), gps.location.lng(),
            missionWaypoints[0].latitude, missionWaypoints[0].longitude)) {
        reportMissionStartBlock(6, "CAPTURED_HOME_ROUTE_OUTSIDE_BOUNDARY");
        missionStartBlockedLogged = true;
        return;
      }
      float startingHeading = 0.0f;
      if (!magnetometerCalibrationLoaded || !magnetometerAvailable || !readMissionHeadingDegrees(startingHeading)) {
        const uint32_t headingAge = missionHeadingCacheValid
          ? millis() - missionHeadingCacheUpdatedMs : UINT32_MAX;
        recordLogLine(String("MISSION HEADING SAMPLE FAILURE context=START calibrated=") +
          String(magnetometerCalibrationLoaded ? 1 : 0) + " available=" +
          String(magnetometerAvailable ? 1 : 0) + " cache_valid=" +
          String(missionHeadingCacheValid ? 1 : 0) + " cache_age_ms=" +
          String(headingAge) + " mag_status=" + String(lastMagnetometerStatus) +
          " not_ready=" + String(magnetometerNotReady) + " i2c_errors=" +
          String(magnetometerI2cErrors));
        reportMissionStartBlock(8, "MAGNETOMETER_CALIBRATION_UNAVAILABLE");
        missionStartBlockedLogged = true;
        return;
      }
      elrsArmBlockedByWifiLogged = false;
      elrsArmInterlockBlockLogged = false;
      elrsArmReady = false;
      manualLinkLossStop = false;
      missionExecutionActive = true;
      missionReturningHome = false;
      missionCheckpointActive = false;
      missionLowBatterySinceMs = 0;
      missionCurrentWaypoint = 0;
      missionCompletedWaypoints = 0;
      missionRunStatus = "STARTING";
      missionStartBlockCode = 0;
      missionStopReason = "";
      missionStartBlockedLogged = false;
      if (captureMissionHome) {
        missionPlannedHomeBeforeRun = plannedMissionHome;
        missionHomeCapturedForRun = true;
        missionMetadata.home.latitude = gps.location.lat();
        missionMetadata.home.longitude = gps.location.lng();
        recordLogLine(String("MISSION HOME CAPTURED ") +
          String(missionMetadata.home.latitude, 7) + "," + String(missionMetadata.home.longitude, 7) +
          " PLANNED=" + String(plannedMissionHome.latitude, 7) + "," + String(plannedMissionHome.longitude, 7));
        emitLine(String("TEL EVENT MISSION_HOME_CAPTURED ") +
          String(missionMetadata.home.latitude, 7) + " " + String(missionMetadata.home.longitude, 7));
      }
      strlcpy(runningMissionId, missionId, sizeof(runningMissionId));
      strlcpy(missionLastRunId, missionId, sizeof(missionLastRunId));
      handleControlCommand("CTRL SCIENCE START " + currentScienceTimestamp());
      if (scienceExperimentState != 1) {
        missionExecutionActive = false;
        missionMetadata.home = plannedMissionHome;
        missionHomeCapturedForRun = false;
        missionRunStatus = "IDLE";
        runningMissionId[0] = '\0';
        elrsArmReady = false;
        reportMissionStartBlock(16, "MISSION_SESSION_CREATE_FAILED");
        recordLogLine("MISSION START BLOCKED: SESSION CREATE FAILED");
        return;
      }
      elrsArmed = true;
      controlSource = CONTROL_SOURCE_ELRS;
      controlMode = CONTROL_MODE_GOTO;
      elrs.sendScienceValue("MIS_BLOCK", 0);
      elrs.sendScienceValue("MIS_STATE", missionTelemetryStatusCode());
      lastMissionControlMs = 0;
      lastMissionUpdateMs = 0;
      recordLogLine(String("MISSION START ") + runningMissionId);
      emitLine(String("TEL EVENT MISSION_START ") + runningMissionId);
    }
    return;
  }
  if (missionModeRequested) {
    if (missionExecutionActive) stopMissionExecution("MISSION MODE SWITCH", false);
    missionStartLocked = false;
    missionStartBlockedLogged = false;
    missionStartBlockCode = 0;
    elrs.sendScienceValue("MIS_BLOCK", 0);
    missionModeRequested = false;
    emitLine("TEL EVENT MISSION_MODE MANUAL");
  }

  const uint16_t armChannel = elrs.channel(4);  // CH5: low disarm, high arm
  if (armChannel < 1300) {
    elrsArmBlockedByWifiLogged = false;
    elrsArmInterlockBlockLogged = false;
    if (elrsArmed || controlSource == CONTROL_SOURCE_ELRS) recordLogLine("ELRS ARM SWITCH LOW; DISARMED");
    if (controlSource == CONTROL_SOURCE_ELRS) stopMotorsForSafety(F("ELRS DISARMED"));
    elrsArmed = false;
    elrsArmReady = true;
    return;
  }
  if (armChannel < 1700 || serviceMode || sdTransferActive) {
    if (controlSource == CONTROL_SOURCE_ELRS) stopMotorsForSafety(F("ELRS DISARMED"));
    elrsArmed = false;
    return;
  }

  const int lateral = elrsStickPercent(elrs.channel(0));
  const int thrust = elrsStickPercent(elrs.channel(1));
  const int yaw = elrsStickPercent(elrs.channel(3));
  const uint16_t powerChannel = elrs.channel(2);
  if (!elrsArmed) {
    // The switch must first be observed low, with zero power and neutral
    // controls before its next high transition can take motor authority.
    if (controlSource == CONTROL_SOURCE_WIFI) {
      elrsArmReady = false;
      if (!elrsArmBlockedByWifiLogged) {
        recordLogLine("ELRS ARM BLOCKED: WIFI CONTROL SOURCE ACTIVE");
        elrsArmBlockedByWifiLogged = true;
      }
      return;
    }
    if (!elrsArmReady || powerChannel > 1100 || lateral || thrust || yaw) {
      if (!elrsArmInterlockBlockLogged) {
        recordLogLine("ELRS ARM BLOCKED: CYCLE ARM LOW; POWER MINIMUM; STICKS CENTERED");
        elrsArmInterlockBlockLogged = true;
      }
      return;
    }
    const bool captureHomeOnArm = elrs.channel(8) >= 1700;  // SA / CH9 held while arming.
    if (captureHomeOnArm && (!gps.location.isValid() || gps.location.age() > 1500 ||
        !gps.satellites.isValid() || gps.satellites.value() < 5 ||
        !gps.hdop.isValid() || gps.hdop.hdop() > 3.0)) {
      if (!elrsArmInterlockBlockLogged) {
        emitLine("TEL EVENT MANUAL_ARM_BLOCKED HOME_GPS_QUALITY");
        recordLogLine("MANUAL ARM BLOCKED: HOME CAPTURE NEEDS GPS FIX");
        elrsArmInterlockBlockLogged = true;
      }
      return;
    }
    elrsArmed = true;
    elrsArmBlockedByWifiLogged = false;
    elrsArmInterlockBlockLogged = false;
    elrsArmReady = false;
    manualLinkLossStop = false;
    controlSource = CONTROL_SOURCE_ELRS;
    controlMode = CONTROL_MODE_MANUAL;
    holdPositionEnabled = false;
    manualReturnHomeActive = false;
    // Require both momentary buttons to be released after arming before the
    // deliberate two-button Return Home command can be accepted.
    manualHomeButtonReleasedAfterArm = false;
    manualRthComboStartedMs = 0;
    manualRthComboTriggered = false;
    if (captureHomeOnArm) {
      manualHome.latitude = gps.location.lat();
      manualHome.longitude = gps.location.lng();
      manualHomeAvailable = true;
      const bool homePersisted = saveManualHomeToPreferences(manualHome);
      emitLine(String("TEL STATUS MANUAL_HOME SET ") + String(manualHome.latitude, 7) + " " + String(manualHome.longitude, 7));
      emitLine(String("TEL STATUS MANUAL_HOME_STORAGE ") + (homePersisted ? "SAVED NVS" : "SAVE FAILED"));
      emitLine("TEL EVENT MANUAL_HOME_CAPTURED");
      if (!homePersisted) emitLine("TEL EVENT MANUAL_HOME_NOT_PERSISTED");
      recordLogLine(String("MANUAL HOME CAPTURED ") + String(manualHome.latitude, 7) + "," + String(manualHome.longitude, 7));
      if (!homePersisted) recordLogLine("MANUAL HOME PERSISTENCE FAILED");
    }
    lastElrsAppliedRcMs = 0;
    recordLogLine("ELRS CONTROL ARMED");
  }
  const uint16_t homeButtonChannel = elrs.channel(8);  // SA / CH9: hold while arming to capture manual Home.
  const uint16_t rthConfirmChannel = elrs.channel(9);  // SD / CH10: hold with SA to confirm Return Home.
  const bool homeButtonPressed = homeButtonChannel >= 1500;
  const bool rthConfirmPressed = rthConfirmChannel >= 1500;
  const bool rthComboPressed = homeButtonPressed && rthConfirmPressed;
  if (!homeButtonPressed && !rthConfirmPressed) {
    manualHomeButtonReleasedAfterArm = true;
    manualRthComboStartedMs = 0;
    manualRthComboTriggered = false;
  } else if (!rthComboPressed) {
    manualRthComboStartedMs = 0;
  } else if (manualHomeButtonReleasedAfterArm && manualHomeAvailable && !manualReturnHomeActive && !manualRthComboTriggered) {
    const uint32_t now = millis();
    if (manualRthComboStartedMs == 0) manualRthComboStartedMs = now;
    if (now - manualRthComboStartedMs >= 1000) {
      manualRthComboTriggered = true;
      manualHomeButtonReleasedAfterArm = false;
      if (!gps.location.isValid() || gps.location.age() > 1500 || !gps.satellites.isValid() ||
          gps.satellites.value() < 5 || !gps.hdop.isValid() || gps.hdop.hdop() > 3.0) {
        emitLine("TEL EVENT MANUAL_RTH_BLOCKED GPS_QUALITY");
        recordLogLine("MANUAL RETURN HOME BLOCKED: GPS QUALITY");
      } else {
        float currentHeading = 0.0f;
        if (!magnetometerCalibrationLoaded || !magnetometerAvailable || !readMissionHeadingDegrees(currentHeading)) {
          emitLine("TEL EVENT MANUAL_RTH_BLOCKED HEADING_UNAVAILABLE");
          recordLogLine("MANUAL RETURN HOME BLOCKED: HEADING UNAVAILABLE");
        } else {
          manualReturnHomeActive = true;
          lastManualReturnUpdateMs = 0;
          controlMode = CONTROL_MODE_GOTO;
          emitLine("TEL EVENT MANUAL_RETURN_HOME STARTED");
          recordLogLine("MANUAL RETURN HOME STARTED");
        }
      }
    }
  }
  if (manualReturnHomeActive) return;
  if (controlSource != CONTROL_SOURCE_ELRS || elrs.lastRcMs() == lastElrsAppliedRcMs) return;
  lastElrsAppliedRcMs = elrs.lastRcMs();
  const int powerPercent = constrain((static_cast<int>(powerChannel) - 1000) / 10, 0, 100);
  const float scale = (powerPercent * ELRS_MOTOR_POWER_LIMIT_PERCENT) / 10000.0f;
  applyMotionCommand(static_cast<int>(roundf(lateral * scale)),
                     static_cast<int>(roundf(thrust * scale)),
                     applyManualHeadingAssist(yaw, static_cast<int>(roundf(yaw * scale)),
                       static_cast<int>(roundf(lateral * scale)),
                       static_cast<int>(roundf(thrust * scale)), scale, now), true);
  controlMode = CONTROL_MODE_MANUAL;
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

bool missionCoordinateInsideBoundary(double latitude, double longitude) {
  if (missionMetadata.boundaryCount < 3) return false;
  bool inside = false;
  for (uint8_t i = 0, j = missionMetadata.boundaryCount - 1; i < missionMetadata.boundaryCount; j = i++) {
    const double yi = missionMetadata.boundary[i].latitude;
    const double xi = missionMetadata.boundary[i].longitude;
    const double yj = missionMetadata.boundary[j].latitude;
    const double xj = missionMetadata.boundary[j].longitude;
    const double cross = (longitude - xi) * (yj - yi) - (latitude - yi) * (xj - xi);
    if (fabs(cross) < 1e-9 && longitude >= min(xi, xj) - 1e-9 && longitude <= max(xi, xj) + 1e-9 &&
        latitude >= min(yi, yj) - 1e-9 && latitude <= max(yi, yj) + 1e-9) return true;
    const bool crosses = ((yi > latitude) != (yj > latitude)) &&
      (longitude < (xj - xi) * (latitude - yi) / (yj - yi) + xi);
    if (crosses) inside = !inside;
  }
  return inside;
}

bool missionDirectReturnPathIsSafe(double latitude, double longitude) {
  if (!gps.location.isValid() || !missionMetadata.hasHome || missionMetadata.boundaryCount < 3) return false;
  const double distance = TinyGPSPlus::distanceBetween(latitude, longitude,
    missionMetadata.home.latitude, missionMetadata.home.longitude);
  if (!isfinite(distance) || distance > 10000.0) return false;
  const uint16_t steps = static_cast<uint16_t>(max(1.0, ceil(distance / 5.0)));
  bool enteredBoundary = missionCoordinateInsideBoundary(latitude, longitude);
  for (uint16_t i = 1; i <= steps; ++i) {
    const double fraction = static_cast<double>(i) / steps;
    const double testLat = latitude + (missionMetadata.home.latitude - latitude) * fraction;
    const double testLon = longitude + (missionMetadata.home.longitude - longitude) * fraction;
    const bool inside = missionCoordinateInsideBoundary(testLat, testLon);
    if (inside) enteredBoundary = true;
    else if (enteredBoundary && i < steps) return false;
  }
  return true;
}

bool missionSegmentInsideBoundary(double fromLatitude, double fromLongitude, double toLatitude, double toLongitude) {
  const double distance = TinyGPSPlus::distanceBetween(fromLatitude, fromLongitude, toLatitude, toLongitude);
  if (!isfinite(distance) || distance > 10000.0) return false;
  const uint16_t steps = static_cast<uint16_t>(max(1.0, ceil(distance / 5.0)));
  for (uint16_t i = 0; i <= steps; ++i) {
    const double fraction = static_cast<double>(i) / steps;
    if (!missionCoordinateInsideBoundary(fromLatitude + (toLatitude - fromLatitude) * fraction,
        fromLongitude + (toLongitude - fromLongitude) * fraction)) return false;
  }
  return true;
}

bool calculateTiltHeadingDegrees(const MagnetometerReadings& mag, const ImuReadings& imu,
                                 float& headingDegrees) {
  const float mx = (static_cast<float>(mag.x) - magnetometerOffsetX) * magnetometerScaleX;
  const float my = (static_cast<float>(mag.y) - magnetometerOffsetY) * magnetometerScaleY;
  const float mz = static_cast<float>(mag.z);
  const float roll = atan2f(imu.accelYG, imu.accelZG);
  const float pitch = atan2f(-imu.accelXG,
    sqrtf(imu.accelYG * imu.accelYG + imu.accelZG * imu.accelZG));
  const float horizontalX = mx * cosf(pitch) + mz * sinf(pitch);
  const float horizontalY = mx * sinf(roll) * sinf(pitch) + my * cosf(roll)
    - mz * sinf(roll) * cosf(pitch);
  headingDegrees = atan2f(horizontalY, horizontalX) * 180.0f / PI;
  if (headingDegrees < 0.0f) headingDegrees += 360.0f;
  return isfinite(headingDegrees);
}

bool readHeadingDegrees(float& headingDegrees, bool requireCalibration) {
  if (!magnetometerAvailable || (requireCalibration && !magnetometerCalibrationLoaded)) return false;
  MagnetometerReadings mag = {};
  ImuReadings imu = {};
  if (!readQmc5883(mag) || !readMpu6050(imu)) return false;
  return calculateTiltHeadingDegrees(mag, imu, headingDegrees);
}

bool readMissionHeadingDegrees(float& headingDegrees) {
  if (!magnetometerCalibrationLoaded || !magnetometerAvailable) {
    missionHeadingCacheValid = false;
    return false;
  }
  if (readHeadingDegrees(headingDegrees, true)) {
    missionHeadingCacheDegrees = headingDegrees;
    missionHeadingCacheUpdatedMs = millis();
    missionHeadingCacheValid = true;
    return true;
  }
  // A not-ready sample is normal between the magnetometer's 50 Hz updates.
  // Reuse only a recent heading; readQmc5883 clears magnetometerAvailable on
  // an I2C failure, which takes the fail-closed path above.
  if (!magnetometerAvailable || !missionHeadingCacheValid ||
      millis() - missionHeadingCacheUpdatedMs > MISSION_HEADING_MAX_AGE_MS) {
    missionHeadingCacheValid = false;
    return false;
  }
  headingDegrees = missionHeadingCacheDegrees;
  return true;
}

bool readManualAssistHeadingDegrees(float& headingDegrees) {
  return readHeadingDegrees(headingDegrees, false);
}

int applyManualHeadingAssist(int stickYaw, int scaledYaw, int scaledLateral,
                             int scaledThrust, float powerScale, uint32_t now) {
  if (!manualAssistRequested) {
    manualAssistTargetValid = false;
    manualAssistYawing = false;
    manualAssistLastYawCorrection = 0.0f;
    manualAssistHeadingAvailable = false;
    return scaledYaw;
  }

  if (abs(stickYaw) > 5) {
    manualAssistYawing = true;
    manualAssistTargetValid = false;
    manualAssistLastYawCorrection = 0.0f;
    if (lastManualAssistUpdateMs == 0 || now - lastManualAssistUpdateMs >= MANUAL_ASSIST_UPDATE_INTERVAL_MS) {
      float heading = 0.0f;
      if (readManualAssistHeadingDegrees(heading)) {
        manualAssistTargetHeadingDegrees = heading;
        manualAssistHeadingAvailable = true;
      } else {
        manualAssistHeadingAvailable = false;
      }
      lastManualAssistUpdateMs = now;
    }
    return scaledYaw;
  }

  float heading = 0.0f;
  if (manualAssistYawing || ((scaledLateral == 0 && scaledThrust == 0) || powerScale <= 0.0f)) {
    manualAssistYawing = false;
    manualAssistLastYawCorrection = 0.0f;
    if (lastManualAssistUpdateMs == 0 || now - lastManualAssistUpdateMs >= MANUAL_ASSIST_UPDATE_INTERVAL_MS) {
      if (readManualAssistHeadingDegrees(heading)) {
        manualAssistTargetHeadingDegrees = heading;
        manualAssistTargetValid = true;
        manualAssistHeadingAvailable = true;
      } else {
        manualAssistTargetValid = false;
        manualAssistHeadingAvailable = false;
      }
      lastManualAssistUpdateMs = now;
    }
    return 0;
  }

  if (now - lastManualAssistUpdateMs < MANUAL_ASSIST_UPDATE_INTERVAL_MS) {
    return constrain(static_cast<int>(roundf(manualAssistLastYawCorrection * powerScale)),
                     -MANUAL_ASSIST_MAX_YAW_PERCENT, MANUAL_ASSIST_MAX_YAW_PERCENT);
  }
  lastManualAssistUpdateMs = now;
  if (!readManualAssistHeadingDegrees(heading)) {
    manualAssistTargetValid = false;
    manualAssistLastYawCorrection = 0.0f;
    manualAssistHeadingAvailable = false;
    return 0;
  }
  manualAssistHeadingAvailable = true;
  if (!manualAssistTargetValid) {
    manualAssistTargetHeadingDegrees = heading;
    manualAssistTargetValid = true;
    manualAssistLastYawCorrection = 0.0f;
    return 0;
  }

  float headingError = manualAssistTargetHeadingDegrees - heading;
  while (headingError > 180.0f) headingError -= 360.0f;
  while (headingError < -180.0f) headingError += 360.0f;
  if (fabsf(headingError) <= MANUAL_ASSIST_HEADING_DEADBAND_DEG) {
    manualAssistLastYawCorrection = 0.0f;
  } else {
    manualAssistLastYawCorrection = constrain(
      headingError * MANUAL_ASSIST_YAW_GAIN_PERCENT_PER_DEG,
      -MANUAL_ASSIST_MAX_YAW_PERCENT, MANUAL_ASSIST_MAX_YAW_PERCENT);
  }
  return constrain(static_cast<int>(roundf(manualAssistLastYawCorrection * powerScale)),
                   -MANUAL_ASSIST_MAX_YAW_PERCENT, MANUAL_ASSIST_MAX_YAW_PERCENT);
}

void updateManualReturnHome(uint32_t now) {
  if (!manualReturnHomeActive) return;
  if (lastManualReturnUpdateMs != 0 && now - lastManualReturnUpdateMs < 200) return;
  lastManualReturnUpdateMs = now;
  if (!elrsArmed || controlSource != CONTROL_SOURCE_ELRS || !elrs.rcFresh(now) || missionModeRequested || serviceMode || sdTransferActive) {
    stopMotorsForSafety(F("MANUAL RETURN HOME INTERLOCK"));
    emitLine("TEL EVENT MANUAL_RETURN_HOME STOPPED CONTROL_INTERLOCK");
    return;
  }
  if (!manualHomeAvailable || !gps.location.isValid() || gps.location.age() > 1500 ||
      !gps.satellites.isValid() || gps.satellites.value() < 5 ||
      !gps.hdop.isValid() || gps.hdop.hdop() > 3.0) {
    stopMotorsForSafety(F("MANUAL RETURN HOME GPS"));
    emitLine("TEL EVENT MANUAL_RETURN_HOME STOPPED GPS_QUALITY");
    return;
  }
  float heading = 0.0f;
  if (!readMissionHeadingDegrees(heading)) {
    stopMotorsForSafety(F("MANUAL RETURN HOME HEADING"));
    emitLine("TEL EVENT MANUAL_RETURN_HOME STOPPED HEADING_UNAVAILABLE");
    return;
  }
  const double latitude = gps.location.lat();
  const double longitude = gps.location.lng();
  const double distanceMeters = TinyGPSPlus::distanceBetween(latitude, longitude,
    manualHome.latitude, manualHome.longitude);
  if (!isfinite(distanceMeters)) {
    stopMotorsForSafety(F("MANUAL RETURN HOME NAVIGATION"));
    emitLine("TEL EVENT MANUAL_RETURN_HOME STOPPED NAVIGATION_INVALID");
    return;
  }
  if (distanceMeters <= 5.0) {
    stopMotorsForSafety(F("MANUAL HOME REACHED"));
    emitLine(String("TEL STATUS MANUAL_HOME SET ") + String(manualHome.latitude, 7) + " " + String(manualHome.longitude, 7));
    emitLine("TEL EVENT MANUAL_RETURN_HOME REACHED");
    recordLogLine("MANUAL RETURN HOME REACHED");
    return;
  }
  const double bearing = TinyGPSPlus::courseTo(latitude, longitude,
    manualHome.latitude, manualHome.longitude);
  float headingError = static_cast<float>(bearing) - heading;
  while (headingError > 180.0f) headingError -= 360.0f;
  while (headingError < -180.0f) headingError += 360.0f;
  const int yaw = constrain(static_cast<int>(roundf(headingError * 0.55f)), -35, 35);
  const int thrust = fabsf(headingError) > 50.0f ? 0 : 25;
  applyMotionCommand(0, thrust, yaw);
  controlMode = CONTROL_MODE_GOTO;
  if (now - lastManualReturnTelemetryMs >= 1000) {
    lastManualReturnTelemetryMs = now;
    emitLine(String("TEL STATUS MANUAL_HOME RETURNING ") + String(distanceMeters, 1) + "M");
  }
}

void beginMissionReturnHome(const String& reason) {
  if (!missionExecutionActive || !missionMetadata.hasHome) return;
  const double latitude = gps.location.lat();
  const double longitude = gps.location.lng();
  missionReturnWaypointIndex = -1;
  if (!missionDirectReturnPathIsSafe(latitude, longitude)) {
    if (!missionCoordinateInsideBoundary(latitude, longitude) || missionCurrentWaypoint == 0) {
      stopMissionExecution("RETURN_HOME_PATH_UNSAFE", false);
      return;
    }
    const MissionWaypoint& prior = missionWaypoints[missionCurrentWaypoint - 1];
    if (!missionSegmentInsideBoundary(latitude, longitude, prior.latitude, prior.longitude)) {
      stopMissionExecution("RETURN_HOME_PATH_UNSAFE", false);
      return;
    }
    missionReturnWaypointIndex = static_cast<int16_t>(missionCurrentWaypoint - 1);
  }
  stopAllMotors();
  missionReturningHome = true;
  missionCheckpointActive = false;
  missionRunStatus = "RETURNING_HOME";
  elrs.sendScienceValue("MIS_STATE", missionTelemetryStatusCode());
  missionStopReason = reason;
  if (audioRecordingEnabled) handleControlCommand("CTRL AUDIO RECORD STOP");
  recordLogLine(String("MISSION RETURN HOME reason=") + reason);
  emitLine(String("TEL EVENT MISSION_RETURN_HOME ") + reason);
}

void stopMissionExecution(const String& reason, bool completed) {
  const bool wasRunning = missionExecutionActive;
  const uint8_t finalIndex = missionCurrentWaypoint;
  const uint8_t finalCompleted = missionCompletedWaypoints;
  const uint8_t percent = missionWaypointCount ? (finalCompleted * 100U / missionWaypointCount) : 0;
  String reasonToken = reason;
  reasonToken.replace(" ", "_");
  stopAllMotors();
  if (scienceExperimentState == 1) {
    if (audioRecordingEnabled) handleControlCommand("CTRL AUDIO RECORD STOP");
    handleControlCommand("CTRL SCIENCE STOP " + currentScienceTimestamp());
  }
  if (scienceExperimentState == 2) handleControlCommand("CTRL SCIENCE SAVE " + currentScienceTimestamp());
  missionExecutionActive = false;
  missionReturningHome = false;
  missionReturnWaypointIndex = -1;
  missionCheckpointActive = false;
  missionLowBatterySinceMs = 0;
  missionStartLocked = true;
  elrsArmed = false;
  elrsArmReady = false;
  controlSource = CONTROL_SOURCE_NONE;
  controlMode = CONTROL_MODE_IDLE;
  holdPositionEnabled = false;
  targetLocationSet = false;
  const bool routeComplete = completed && reason == "ROUTE_COMPLETE";
  const bool returnedHome = completed && reason == "HOME_REACHED";
  missionRunStatus = routeComplete ? "COMPLETE" : (returnedHome ? "RETURNED_HOME" : "STOPPED");
  elrs.sendScienceValue("MIS_STATE", missionTelemetryStatusCode());
  missionStopReason = reason;
  runningMissionId[0] = '\0';
  if (wasRunning) {
    const char* finalLabel = routeComplete ? "COMPLETE" : (returnedHome ? "RETURNED_HOME" : "STOPPED");
    const char* eventLabel = routeComplete ? "TEL EVENT MISSION_COMPLETE " :
      (returnedHome ? "TEL EVENT MISSION_RETURNED_HOME " : "TEL EVENT MISSION_STOP ");
    recordLogLine(String(routeComplete || returnedHome ? "MISSION COMPLETE " : "MISSION STOP ") + missionLastRunId + " reason=" + reason);
    emitLine(String(eventLabel) + reason);
    emitLine(String("TEL MISSION PROGRESS ") + missionLastRunId + " " + finalLabel + " " +
      String(finalIndex) + " " + String(missionWaypointCount) + " " + String(finalCompleted) + " -1 -1 " + String(percent) + " " + reasonToken);
    emitLine(String("TEL STATUS MISSION ") + String(missionWaypointCount) + " UPLOAD READY LOADED " +
      String(missionId[0] ? missionId : "NONE") + " RUNNING NONE");
  }
  if (missionHomeCapturedForRun) {
    missionMetadata.home = missionPlannedHomeBeforeRun;
    missionHomeCapturedForRun = false;
  }
}

void updateMissionExecution(uint32_t now) {
  if (!missionExecutionActive) return;
  if (lastMissionUpdateMs != 0 && now - lastMissionUpdateMs < 200) return;
  lastMissionUpdateMs = now;
  if (serviceMode || sdTransferActive) {
    stopMissionExecution("SERVICE_MODE", false);
    return;
  }
  if (scienceExperimentState != 1) {
    stopMissionExecution("MISSION_SESSION_UNAVAILABLE", false);
    return;
  }
  if (!missionReturningHome && !missionModeRequested && elrs.rcFresh(now) && elrsArmed) {
    stopMissionExecution("MISSION_MODE_SWITCHED_TO_MANUAL", false);
    return;
  }
  const bool missionRadioLost = !elrs.rcFresh(now) || !elrsArmed;
  if (!missionReturningHome && missionRadioLost) {
    const bool gpsAllowsReturn = gps.location.isValid() && gps.location.age() <= 1500 &&
      gps.satellites.isValid() && gps.satellites.value() >= 5 &&
      gps.hdop.isValid() && gps.hdop.hdop() <= 3.0 && missionMetadata.hasHome;
    if (missionMetadata.linkLossAction == 1 && gpsAllowsReturn) {
      recordLogLine("MISSION RC LOST: RETURN-HOME ACTION SELECTED");
      beginMissionReturnHome("ELRS LINK LOST");
      if (!missionExecutionActive) return;
    } else {
      stopMissionExecution(gpsAllowsReturn ? "RADIO_CONTROL_LOST" : "RADIO_CONTROL_LOST_GPS_UNAVAILABLE", false);
      return;
    }
  }
  if (!gps.location.isValid() || gps.location.age() > 1500 || !gps.satellites.isValid() ||
      gps.satellites.value() < 5 || !gps.hdop.isValid() || gps.hdop.hdop() > 3.0) {
    stopMissionExecution("GPS_QUALITY_LOST", false);
    return;
  }

  float heading = 0.0f;
  if (!readMissionHeadingDegrees(heading)) {
    const uint32_t headingAge = missionHeadingCacheValid
      ? now - missionHeadingCacheUpdatedMs : UINT32_MAX;
    recordLogLine(String("MISSION HEADING SAMPLE FAILURE context=EXECUTION calibrated=") +
      String(magnetometerCalibrationLoaded ? 1 : 0) + " available=" +
      String(magnetometerAvailable ? 1 : 0) + " cache_valid=" +
      String(missionHeadingCacheValid ? 1 : 0) + " cache_age_ms=" +
      String(headingAge) + " mag_status=" + String(lastMagnetometerStatus) +
      " not_ready=" + String(magnetometerNotReady) + " i2c_errors=" +
      String(magnetometerI2cErrors));
    stopMissionExecution("HEADING_UNAVAILABLE", false);
    return;
  }

  const double latitude = gps.location.lat();
  const double longitude = gps.location.lng();
  if (!missionCoordinateInsideBoundary(latitude, longitude) && !missionReturningHome) {
    if (missionMetadata.boundaryBreachAction == 1 && missionMetadata.hasHome) beginMissionReturnHome("BOUNDARY_BREACH");
    else { stopMissionExecution("BOUNDARY_BREACH", false); return; }
    if (!missionExecutionActive) return;
  }
  const bool batterySampleFresh = latestBatteryVoltageValid &&
    now - latestBatterySampleMs <= (ELRS_BATTERY_INTERVAL_MS * 3);
  const int batteryPercent = batterySampleFresh ? estimateBatteryPercent(latestBatteryVolts) : -1;
  if (!missionReturningHome && batterySampleFresh && batteryPercent >= 0 &&
      batteryPercent <= missionMetadata.batteryThresholdPercent) {
    if (missionLowBatterySinceMs == 0) missionLowBatterySinceMs = now;
    if (now - missionLowBatterySinceMs >= MISSION_LOW_BATTERY_SUSTAIN_MS) {
      recordLogLine(String("MISSION LOW BATTERY SUSTAINED percent=") +
        String(batteryPercent) + " threshold=" + String(missionMetadata.batteryThresholdPercent));
      if (missionMetadata.lowBatteryAction == 1 && missionMetadata.hasHome) beginMissionReturnHome("LOW_BATTERY");
      else { stopMissionExecution("LOW_BATTERY", false); return; }
      missionLowBatterySinceMs = 0;
      if (!missionExecutionActive) return;
    }
  } else if (!batterySampleFresh || batteryPercent >
      missionMetadata.batteryThresholdPercent + MISSION_LOW_BATTERY_HYSTERESIS_PERCENT || missionReturningHome) {
    missionLowBatterySinceMs = 0;
  }

  MissionCoordinate target = {};
  uint8_t activeIndex = missionCurrentWaypoint;
  if (missionReturningHome) {
    if (missionReturnWaypointIndex >= 0) {
      const MissionWaypoint& waypoint = missionWaypoints[missionReturnWaypointIndex];
      target = {waypoint.latitude, waypoint.longitude};
      activeIndex = static_cast<uint8_t>(missionReturnWaypointIndex);
    } else {
      target = missionMetadata.home;
    }
  } else if (missionCurrentWaypoint < missionWaypointCount) {
    const MissionWaypoint& waypoint = missionWaypoints[missionCurrentWaypoint];
    target = {waypoint.latitude, waypoint.longitude};
  } else {
    stopMissionExecution("ROUTE_COMPLETE", true);
    return;
  }

  const double distanceMeters = TinyGPSPlus::distanceBetween(latitude, longitude, target.latitude, target.longitude);
  double distanceHomeMeters = TinyGPSPlus::distanceBetween(latitude, longitude,
    missionMetadata.home.latitude, missionMetadata.home.longitude);
  if (!isfinite(distanceMeters) || !isfinite(distanceHomeMeters)) {
    stopMissionExecution("NAVIGATION_INVALID", false);
    return;
  }
  missionDistanceToWaypointMeters = static_cast<float>(distanceMeters);
  missionDistanceFromHomeMeters = static_cast<float>(distanceHomeMeters);
  missionPercentComplete = missionWaypointCount ? (missionCompletedWaypoints * 100U / missionWaypointCount) : 0;
  String reasonToken = missionStopReason.length() ? missionStopReason : "NONE";
  reasonToken.replace(" ", "_");

  if (distanceMeters <= 5.0) {
    stopAllMotors();
    if (missionReturningHome) {
      if (missionReturnWaypointIndex >= 0) {
        --missionReturnWaypointIndex;
        if (missionReturnWaypointIndex >= 0) {
          const MissionWaypoint& next = missionWaypoints[missionReturnWaypointIndex];
          if (!missionSegmentInsideBoundary(latitude, longitude, next.latitude, next.longitude)) {
            stopMissionExecution("RETURN_HOME_PATH_UNSAFE", false);
            return;
          }
          return;
        }
      }
      if (!missionDirectReturnPathIsSafe(latitude, longitude)) {
        stopMissionExecution("RETURN_HOME_PATH_UNSAFE", false);
        return;
      }
      stopMissionExecution("HOME_REACHED", true);
      return;
    }
    const MissionWaypoint& waypoint = missionWaypoints[missionCurrentWaypoint];
    const bool shouldCollect = waypoint.collectData && waypoint.holdSeconds > 0;
    if (shouldCollect && !missionCheckpointActive) {
      missionCheckpointActive = true;
      missionCheckpointStartedMs = now;
      missionRunStatus = "COLLECTING";
      elrs.sendScienceValue("MIS_STATE", missionTelemetryStatusCode());
      if (scienceExperimentState == 1 && audioInputAvailable) handleControlCommand("CTRL AUDIO RECORD START");
    }
    if (missionCheckpointActive && now - missionCheckpointStartedMs < static_cast<uint32_t>(waypoint.holdSeconds) * 1000UL) {
      // Hold position by commanding zero; this hull has no station-keeping actuator.
      if (now - lastMissionTelemetryMs >= 1000) {
        lastMissionTelemetryMs = now;
        const uint8_t percent = missionWaypointCount ? (missionCompletedWaypoints * 100U / missionWaypointCount) : 0;
        emitLine(String("TEL MISSION PROGRESS ") + missionLastRunId + " COLLECTING " + String(activeIndex) + " " +
          String(missionWaypointCount) + " " + String(missionCompletedWaypoints) + " 0 " + String(distanceHomeMeters, 1) + " " + String(percent) + " CHECKPOINT");
      }
      return;
    }
    if (missionCheckpointActive) {
      if (audioRecordingEnabled) handleControlCommand("CTRL AUDIO RECORD STOP");
      missionCheckpointActive = false;
    }
    ++missionCompletedWaypoints;
    ++missionCurrentWaypoint;
    missionRunStatus = "MOVING";
    if (missionCurrentWaypoint >= missionWaypointCount) {
      stopMissionExecution("ROUTE_COMPLETE", true);
      return;
    }
  }

  const double bearing = TinyGPSPlus::courseTo(latitude, longitude, target.latitude, target.longitude);
  float headingError = static_cast<float>(bearing) - heading;
  while (headingError > 180.0f) headingError -= 360.0f;
  while (headingError < -180.0f) headingError += 360.0f;
  const int yaw = constrain(static_cast<int>(roundf(headingError * 0.55f)), -35, 35);
  const uint8_t speedPercent = missionReturningHome ? 25 : missionWaypoints[missionCurrentWaypoint].speedPercent;
  const int thrust = fabsf(headingError) > 50.0f ? 0 : speedPercent;
  if (now - lastMissionControlMs >= 200) {
    lastMissionControlMs = now;
    applyMotionCommand(0, thrust, yaw);
  }
  targetLatitude = target.latitude;
  targetLongitude = target.longitude;
  targetLocationSet = true;
  controlMode = CONTROL_MODE_GOTO;
  const String nextMissionStatus = missionReturningHome ? "RETURNING_HOME" : "MOVING";
  if (missionRunStatus != nextMissionStatus) {
    missionRunStatus = nextMissionStatus;
    elrs.sendScienceValue("MIS_STATE", missionTelemetryStatusCode());
  }

  if (now - lastMissionTelemetryMs >= 1000) {
    lastMissionTelemetryMs = now;
    const uint8_t percent = missionWaypointCount ? (missionCompletedWaypoints * 100U / missionWaypointCount) : 0;
    emitLine(String("TEL MISSION PROGRESS ") + missionLastRunId + " " + missionRunStatus + " " + String(activeIndex) + " " +
      String(missionWaypointCount) + " " + String(missionCompletedWaypoints) + " " + String(distanceMeters, 1) + " " +
      String(distanceHomeMeters, 1) + " " + String(percent) + " " + reasonToken);
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
      }
      if (computeMotorPwm(i, motors[i], currentDrive) != appliedMotorPwm[i])
        applyMotorOutput(motors[i], currentDrive);
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

  // The conversion register retains the previous channel's result until the
  // new single-shot conversion finishes. Poll OS instead of assuming that a
  // fixed delay is long enough after changing the input multiplexer.
  const uint32_t conversionStartUs = micros();
  bool conversionReady = false;
  while (static_cast<uint32_t>(micros() - conversionStartUs) < 20000UL) {
    Wire.beginTransmission(ADS1115_I2C_ADDRESS);
    Wire.write(0x01);
    if (Wire.endTransmission(false) != 0 ||
        Wire.requestFrom(static_cast<int>(ADS1115_I2C_ADDRESS), 2, 1) != 2) {
      ads1115Available = false;
      return false;
    }
    const uint16_t status = static_cast<uint16_t>((Wire.read() << 8) | Wire.read());
    if (status & 0x8000) {
      conversionReady = true;
      break;
    }
    delayMicroseconds(100);
  }
  if (!conversionReady) {
    return false;
  }

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

void emitWaterChemistryTelemetry() {
  const float tdsVoltage = readAds1115AverageVoltage(ADS1115_TDS_CHANNEL, 1);
  const float turbidityAdcVoltage = readAds1115AverageVoltage(ADS1115_TURBIDITY_CHANNEL, 1);
  emitLine(isnan(tdsVoltage)
    ? "TEL SCI TDS_V UNKNOWN"
    : "TEL SCI TDS_V " + String(tdsVoltage, 3));
  emitLine(isnan(turbidityAdcVoltage)
    ? "TEL SCI TURBIDITY_V UNKNOWN"
    : "TEL SCI TURBIDITY_V " + String(turbidityAdcVoltage / TURBIDITY_INPUT_DIVIDER_RATIO, 3));
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

bool updateLatestBatteryVoltage(float volts) {
  // All battery state is stored as whole-pack voltage. This rejects common
  // per-cell-scale values while allowing the observed loaded sag near 8 V.
  if (!isfinite(volts) || volts < BATTERY_MIN_VALID_PACK_VOLTAGE ||
      volts > BATTERY_MAX_VALID_PACK_VOLTAGE) return false;
  const uint32_t now = millis();
  latestBatteryVolts = volts;
  latestBatteryVoltageValid = true;
  latestBatterySampleMs = now;
  if (!motorCommandActive()) {
    latestRestingBatteryVolts = volts;
    latestRestingBatteryVoltageValid = true;
    latestRestingBatterySampleMs = now;
  }
  return true;
}

int estimateBatteryPercent(float batteryVoltage) {
  const uint32_t now = millis();
  if (motorCommandActive() && latestRestingBatteryVoltageValid &&
      now - latestRestingBatterySampleMs <= BATTERY_RESTING_PERCENT_MAX_AGE_MS) {
    batteryVoltage = latestRestingBatteryVolts;
  }
  if (!isfinite(batteryVoltage) || batteryVoltage < BATTERY_MIN_VALID_PACK_VOLTAGE ||
      batteryVoltage > BATTERY_MAX_VALID_PACK_VOLTAGE) return 0;
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

float currentAmpsFromSensorVoltage(float sensorVoltage) {
  if (isnan(sensorVoltage)) {
    return 0.0f;
  }
  // The installed sensor output falls with battery-feed current. Startup
  // calibration captures the powered electronics, which already draw 0.21 A.
  float loadChangeAmps = (currentSensorZeroVoltage - sensorVoltage) / ACS712_SENSITIVITY_VOLTS_PER_AMP;

  if (fabsf(loadChangeAmps) < CURRENT_SENSOR_NOISE_FLOOR_AMPS) {
    loadChangeAmps = 0.0f;
  }

  return CURRENT_SENSOR_STARTUP_LOAD_AMPS + loadChangeAmps;
}

float updateLatestCurrentSample(float sensorVoltage) {
  const uint32_t now = millis();
  latestCurrentSampleMs = now;
  latestCurrentSampleValid = isfinite(sensorVoltage);
  if (!latestCurrentSampleValid) return NAN;
  latestCurrentAmps = currentAmpsFromSensorVoltage(sensorVoltage);
  return latestCurrentAmps;
}

float readCurrentSensorAmps() {
  const float sensorVoltage = readCurrentSensorVoltage();
  const float amps = updateLatestCurrentSample(sensorVoltage);
  return isfinite(amps) ? amps : 0.0f;
}

void sampleElrsWaterMotion(uint32_t now) {
  if (lastElrsMotionSampleMs != 0 && now - lastElrsMotionSampleMs < ELRS_MOTION_SAMPLE_INTERVAL_MS) return;
  lastElrsMotionSampleMs = now;
  if (elrsMotionWindowStartMs == 0) elrsMotionWindowStartMs = now;

  ImuReadings imu = {};
  if (readMpu6050(imu)) {
    const float accelMagnitude = sqrtf(imu.accelXG * imu.accelXG +
      imu.accelYG * imu.accelYG + imu.accelZG * imu.accelZG);
    const float dynamicAccel = accelMagnitude - 1.0f;
    const float angularRate = sqrtf(imu.gyroXDps * imu.gyroXDps +
      imu.gyroYDps * imu.gyroYDps + imu.gyroZDps * imu.gyroZDps);
    elrsDynamicAccelSquaredSum += dynamicAccel * dynamicAccel;
    elrsAngularRateSquaredSum += angularRate * angularRate;
    ++elrsMotionSampleCount;
  }

  if (now - elrsMotionWindowStartMs < ELRS_MOTION_WINDOW_MS) return;
  if (elrsMotionSampleCount >= 20) {
    const float sampleCount = static_cast<float>(elrsMotionSampleCount);
    elrs.sendScienceValue("SEA_ACC", sqrtf(elrsDynamicAccelSquaredSum / sampleCount));
    elrs.sendScienceValue("SEA_GYR", sqrtf(elrsAngularRateSquaredSum / sampleCount));
  }
  elrsMotionWindowStartMs = now;
  elrsDynamicAccelSquaredSum = 0.0f;
  elrsAngularRateSquaredSum = 0.0f;
  elrsMotionSampleCount = 0;
}

void sendElrsNavigationTelemetry(uint32_t now) {
  if (now - lastElrsNavigationMs < ELRS_NAVIGATION_INTERVAL_MS) return;
  lastElrsNavigationMs = now;
  if (!magnetometerCalibrationLoaded || !magnetometerAvailable) return;

  MagnetometerReadings mag = {};
  ImuReadings imu = {};
  if (!readQmc5883(mag)) return;

  const float mx = (static_cast<float>(mag.x) - magnetometerOffsetX) * magnetometerScaleX;
  const float my = (static_cast<float>(mag.y) - magnetometerOffsetY) * magnetometerScaleY;
  const float mz = static_cast<float>(mag.z);
  float magneticHeading = atan2f(my, mx) * 180.0f / PI;
  if (magneticHeading < 0.0f) magneticHeading += 360.0f;
  elrs.sendScienceValue("MAG_HEAD", magneticHeading);

  // Accelerometer-based tilt compensation is a low-rate snapshot. Dynamic
  // motion can affect this estimate; it is not an attitude stream.
  if (!readMpu6050(imu)) return;
  const float roll = atan2f(imu.accelYG, imu.accelZG);
  const float pitch = atan2f(-imu.accelXG,
    sqrtf(imu.accelYG * imu.accelYG + imu.accelZG * imu.accelZG));
  const float horizontalX = mx * cosf(pitch) + mz * sinf(pitch);
  const float horizontalY = mx * sinf(roll) * sinf(pitch) + my * cosf(roll)
    - mz * sinf(roll) * cosf(pitch);
  float tiltHeading = atan2f(horizontalY, horizontalX) * 180.0f / PI;
  if (tiltHeading < 0.0f) tiltHeading += 360.0f;
  elrs.sendScienceValue("TILT_HEAD", tiltHeading);
}

void sendElrsAttitudeTelemetry(uint32_t now) {
  if (now - lastElrsAttitudeMs < ELRS_ATTITUDE_INTERVAL_MS) return;
  lastElrsAttitudeMs = now;
  if (!magnetometerCalibrationLoaded || !magnetometerAvailable) return;

  MagnetometerReadings mag = {};
  ImuReadings imu = {};
  if (!readQmc5883(mag) || !readMpu6050(imu)) return;

  const float mx = (static_cast<float>(mag.x) - magnetometerOffsetX) * magnetometerScaleX;
  const float my = (static_cast<float>(mag.y) - magnetometerOffsetY) * magnetometerScaleY;
  const float mz = static_cast<float>(mag.z);
  const float roll = atan2f(imu.accelYG, imu.accelZG);
  const float pitch = atan2f(-imu.accelXG,
    sqrtf(imu.accelYG * imu.accelYG + imu.accelZG * imu.accelZG));
  const float horizontalX = mx * cosf(pitch) + mz * sinf(pitch);
  const float horizontalY = mx * sinf(roll) * sinf(pitch) + my * cosf(roll)
    - mz * sinf(roll) * cosf(pitch);
  float headingDegrees = atan2f(horizontalY, horizontalX) * 180.0f / PI;
  if (headingDegrees < 0.0f) headingDegrees += 360.0f;
  lastElrsStatusHeading = static_cast<int>(lroundf(headingDegrees)) % 360;
  lastElrsStatusHeadingMs = now;

  // ELRS converts MAVLink ATTITUDE into standard CRSF attitude sensors.
  // This one-hertz estimate is not a high-rate or acceleration-corrected AHRS.
  elrs.sendAttitude(roll, pitch, headingDegrees * PI / 180.0f);
}

bool loadManualHomeFromPreferences() {
  manualHomeAvailable = false;
  if (!manualHomePreferencesReady ||
      manualHomePreferences.getBytesLength("manualHome") != sizeof(PersistedManualHome)) return false;
  PersistedManualHome stored = {};
  if (manualHomePreferences.getBytes("manualHome", &stored, sizeof(stored)) != sizeof(stored) ||
      stored.magic != MANUAL_HOME_STORAGE_MAGIC) return false;
  const double latitude = stored.latitudeE7 / 10000000.0;
  const double longitude = stored.longitudeE7 / 10000000.0;
  if (!isfinite(latitude) || !isfinite(longitude) || fabs(latitude) > 90.0 || fabs(longitude) > 180.0) return false;
  manualHome.latitude = latitude;
  manualHome.longitude = longitude;
  manualHomeAvailable = true;
  return true;
}

bool saveManualHomeToPreferences(const MissionCoordinate& home) {
  if (!manualHomePreferencesReady || !isfinite(home.latitude) || !isfinite(home.longitude) ||
      fabs(home.latitude) > 90.0 || fabs(home.longitude) > 180.0) return false;
  PersistedManualHome stored = {
    MANUAL_HOME_STORAGE_MAGIC,
    static_cast<int32_t>(lround(home.latitude * 10000000.0)),
    static_cast<int32_t>(lround(home.longitude * 10000000.0))
  };
  return manualHomePreferences.putBytes("manualHome", &stored, sizeof(stored)) == sizeof(stored);
}

bool activeTelemetryHome(MissionCoordinate& home, uint8_t& homeKind) {
  if (missionModeRequested && missionMetadata.hasHome) {
    home = missionMetadata.home;
    homeKind = 2;
    return true;
  }
  if (!missionModeRequested && manualHomeAvailable) {
    home = manualHome;
    homeKind = 1;
    return true;
  }
  home = {};
  homeKind = 0;
  return false;
}

void sendElrsHomeTelemetry() {
  MissionCoordinate home = {};
  uint8_t homeKind = 0;
  const bool hasHome = activeTelemetryHome(home, homeKind);
  const bool changed = homeKind != lastElrsHomeKind ||
    (hasHome && (fabs(home.latitude - lastElrsHomeLatitude) > 0.00000001 ||
                 fabs(home.longitude - lastElrsHomeLongitude) > 0.00000001));
  if (changed) {
    char statusText[51];
    if (hasHome) {
      snprintf(statusText, sizeof(statusText), "HOME %s %.7f %.7f",
        homeKind == 1 ? "MANUAL" : "MISSION", home.latitude, home.longitude);
      lastElrsHomeLatitude = home.latitude;
      lastElrsHomeLongitude = home.longitude;
    } else {
      strlcpy(statusText, "HOME NONE", sizeof(statusText));
      lastElrsHomeLatitude = NAN;
      lastElrsHomeLongitude = NAN;
    }
    elrs.sendStatusText(statusText);
    elrs.sendHomeKind(homeKind);
    lastElrsHomeKind = homeKind;
  }
  elrs.sendHomePosition(hasHome ? home.latitude : 0.0,
                        hasHome ? home.longitude : 0.0, 0.0f);
  if (hasHome && gps.location.isValid() && gps.location.age() < 2000) {
    const double distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(),
      home.latitude, home.longitude);
    if (isfinite(distance)) elrs.sendScienceValue("HOME_DIST", static_cast<float>(distance));
  }
}

void sendWifiHomeTelemetry(uint32_t now) {
  if (telemetryMode != TELEMETRY_MODE_WIFI || now - lastWifiHomeStatusMs < 5000 || !controlClientConnected()) return;
  lastWifiHomeStatusMs = now;
  MissionCoordinate home = {};
  uint8_t homeKind = 0;
  if (!activeTelemetryHome(home, homeKind)) {
    emitLine("TEL STATUS ACTIVE_HOME NONE");
    emitLine("TEL STATUS HOME_DISTANCE UNKNOWN");
    return;
  }
  emitLine(String("TEL STATUS ACTIVE_HOME ") + (homeKind == 1 ? "MANUAL " : "MISSION ") +
    String(home.latitude, 7) + " " + String(home.longitude, 7));
  if (gps.location.isValid() && gps.location.age() < 2000) {
    const double distance = TinyGPSPlus::distanceBetween(gps.location.lat(), gps.location.lng(), home.latitude, home.longitude);
    emitLine(isfinite(distance) ? String("TEL STATUS HOME_DISTANCE ") + String(distance, 1) + " M"
                                : String("TEL STATUS HOME_DISTANCE UNKNOWN"));
  } else {
    emitLine("TEL STATUS HOME_DISTANCE UNKNOWN");
  }
}

// Versioned, bounded status for EdgeTX CRSF passthrough text and the Backpack.
// Fields: armed, manual assist, mission state/percent/waypoint/count,
// session, audio, heading, home distance, GPS fix, home kind, start block.
void sendElrsCompactStatus(uint32_t now) {
  if (now - lastElrsCompactStatusMs < 2000) return;
  lastElrsCompactStatusMs = now;
  MissionCoordinate home = {};
  uint8_t homeKind = 0;
  const bool hasHome = activeTelemetryHome(home, homeKind);
  const bool gpsFix = gps.location.isValid() && gps.location.age() < 2000;
  long homeMeters = -1;
  if (hasHome && gpsFix) {
    const double meters = TinyGPSPlus::distanceBetween(
      gps.location.lat(), gps.location.lng(), home.latitude, home.longitude);
    if (isfinite(meters)) homeMeters = constrain(static_cast<long>(lround(meters)), 0L, 99999L);
  }
  const int heading = lastElrsStatusHeadingMs && now - lastElrsStatusHeadingMs < 3000
    ? lastElrsStatusHeading : -1;
  const int assist = missionModeRequested ? 3 : (!manualAssistRequested ? 0 :
    (manualAssistHeadingAvailable ? 1 : 2));
  char statusText[51];
  snprintf(statusText, sizeof(statusText), "B1,%u,%d,%u,%u,%u,%u,%u,%u,%d,%ld,%u,%u,%u",
    elrsArmed ? 1U : 0U, assist, missionTelemetryStatusCode(),
    static_cast<unsigned>(constrain(static_cast<int>(lroundf(missionPercentComplete)), 0, 100)),
    missionExecutionActive ? static_cast<unsigned>(missionCurrentWaypoint + 1) : 0U,
    static_cast<unsigned>(missionWaypointCount), scienceExperimentState == 1 ? 1U : 0U,
    audioRecordingEnabled ? 1U : 0U, heading, homeMeters, gpsFix ? 1U : 0U,
    static_cast<unsigned>(homeKind), static_cast<unsigned>(missionStartBlockCode));
  elrs.sendStatusText(statusText);
  if (elrsCalibrationStatusCode != 0) {
    char calibrationText[51];
    snprintf(calibrationText, sizeof(calibrationText), "BC1,%u,%u,%u,%u,%lu",
      static_cast<unsigned>(elrsCalibrationStatusCode),
      static_cast<unsigned>(elrsCalibrationTestIndex + 1),
      static_cast<unsigned>(elrsCalibrationCurrentPowerPercent),
      static_cast<unsigned>(elrsCalibrationPowerPercent),
      static_cast<unsigned long>(elrsCalibrationPulseDurationMs / 1000U));
    elrs.sendStatusText(calibrationText);
  }
}

void serviceElrsTelemetry(uint32_t now) {
  if (telemetryMode == TELEMETRY_MODE_WIFI) {
    sendWifiHomeTelemetry(now);
  }
  const bool detailedRadio = telemetryMode == TELEMETRY_MODE_MAVLINK;
  if (detailedRadio) {
    sampleElrsWaterMotion(now);
    elrs.sendLinkAndChannels(now);
  }
  if (now - lastElrsTelemetryMs >= ELRS_TELEMETRY_INTERVAL_MS) {
    lastElrsTelemetryMs = now;
    elrs.sendHeartbeat(elrsArmed && controlSource == CONTROL_SOURCE_ELRS);
    const uint8_t state = static_cast<uint8_t>(controlMode) |
      (static_cast<uint8_t>(controlSource) << 3) |
      (elrsArmed ? 0x20 : 0) |
      (scienceExperimentState == 1 ? 0x40 : 0) |
      (manualLinkLossStop ? 0x80 : 0);
    const int rearPwm = driveValues[0] < 0 ? -appliedMotorPwm[0] : appliedMotorPwm[0];
    const int leftPwm = driveValues[1] < 0 ? -appliedMotorPwm[1] : appliedMotorPwm[1];
    const int rightPwm = driveValues[2] < 0 ? -appliedMotorPwm[2] : appliedMotorPwm[2];
    elrs.sendBuoyState(rearPwm, leftPwm, rightPwm, state);
    elrs.sendBuoyAudioState(audioRecordingEnabled);
    const int assistState = missionModeRequested ? 3 : (!manualAssistRequested ? 0 :
      (manualAssistHeadingAvailable ? 1 : 2));
    if (detailedRadio) elrs.sendScienceValue("MAN_AST", assistState);
  }
  if (now - lastElrsBatteryMs >= ELRS_BATTERY_INTERVAL_MS) {
    lastElrsBatteryMs = now;
    const float battery = readBatterySystemVoltage();
    if (updateLatestBatteryVoltage(battery)) {
      const float current = readCurrentSensorAmps();
      lastStandalonePowerSampleMs = now;
      elrs.sendBattery(battery, current, estimateBatteryPercent(battery));
    }
  }
  if (now - lastElrsGpsMs >= ELRS_GPS_INTERVAL_MS) {
    lastElrsGpsMs = now;
    const bool fixValid = gps.location.isValid() && gps.location.age() < 2000;
    if (fixValid) sendElrsHomeTelemetry();
    const bool altitudeValid = fixValid && gps.altitude.isValid() && gps.altitude.age() < 2000;
    const uint8_t gpsFixType = !fixValid ? 1 : (altitudeValid ? 3 : 2);
    elrs.sendGps(fixValid ? gps.location.lat() : 0.0,
                 fixValid ? gps.location.lng() : 0.0,
                 fixValid && gps.speed.isValid() ? gps.speed.mps() : NAN,
                 fixValid && gps.course.isValid() ? gps.course.deg() : NAN,
                 altitudeValid ? gps.altitude.meters() : NAN,
                 gps.satellites.isValid() ? static_cast<uint8_t>(constrain(gps.satellites.value(), 0UL, 254UL)) : 255,
                 gpsFixType,
                 gps.hdop.isValid() ? static_cast<float>(gps.hdop.hdop()) : NAN,
                 static_cast<uint64_t>(esp_timer_get_time()));
  }
  sendElrsAttitudeTelemetry(now);
  sendElrsCompactStatus(now);
  if (!detailedRadio) return;
  sendElrsNavigationTelemetry(now);
  if (now - lastElrsScienceMs >= ELRS_SCIENCE_INTERVAL_MS) {
    lastElrsScienceMs = now;
    // One small science value per second keeps the Backpack link light.
    // Mission navigation values take priority while a route is active.
    if (elrsCalibrationActive) {
      switch (elrsScienceField++ % 5) {
        case 0: elrs.sendScienceValue("CAL_STAT", 1); break;
        case 1: elrs.sendScienceValue("CAL_STEP", elrsCalibrationTestIndex + 1); break;
        case 2: elrs.sendScienceValue("CAL_PWR", elrsCalibrationCurrentPowerPercent); break;
        case 3: elrs.sendScienceValue("CAL_MAX", elrsCalibrationPowerPercent); break;
        case 4: elrs.sendScienceValue("CAL_DUR", elrsCalibrationPulseDurationMs / 1000U); break;
      }
    } else if (missionExecutionActive) {
      switch (elrsScienceField++ % 6) {
        case 0: elrs.sendScienceValue("MIS_PCT", missionPercentComplete); break;
        case 1: if (isfinite(missionDistanceFromHomeMeters)) elrs.sendScienceValue("DIST_HOME", missionDistanceFromHomeMeters); break;
        case 2: if (isfinite(missionDistanceToWaypointMeters)) elrs.sendScienceValue("DIST_WP", missionDistanceToWaypointMeters); break;
        case 3: elrs.sendScienceValue("MIS_WP", missionCurrentWaypoint + 1); break;
        case 4: elrs.sendScienceValue("MIS_STATE", missionTelemetryStatusCode()); break;
        case 5: elrs.sendScienceValue("MIS_COUNT", missionWaypointCount); break;
      }
    } else switch (elrsScienceField++ % 5) {
      // Keep the terminal/idle mission state visible while rotating sensor fields.
      case 0:
        elrs.sendScienceValue("MIS_STATE", missionTelemetryStatusCode());
        elrs.sendScienceValue("MIS_BLOCK", missionStartBlockCode);
        break;
      case 1:
        if (ds18b20HasReading) elrs.sendScienceValue("WATER_TEMP", ds18b20TemperatureC);
        break;
      case 2:
        if (latestRangeValid) elrs.sendScienceValue("DEPTH_M", latestRangeDistanceMm / 1000.0f);
        break;
      case 3: {
        const float value = readAds1115AverageVoltage(ADS1115_TDS_CHANNEL, 1);
        if (!isnan(value)) elrs.sendScienceValue("TDS_V", value);
        break;
      }
      case 4: {
        const float value = readAds1115AverageVoltage(ADS1115_TURBIDITY_CHANNEL, 1);
        if (!isnan(value)) elrs.sendScienceValue("TURBIDITY", value / TURBIDITY_INPUT_DIVIDER_RATIO);
        break;
      }
    }
  }
}

void emitSensorDiagnostics() {
  pinMode(DS18B20_DATA_PIN, INPUT);

  float currentVoltage = readCurrentSensorVoltage();
  float batteryAdcVoltage = readBatteryAdcVoltage();
  float batteryVoltage = isnan(batteryAdcVoltage) ? 0.0f : batteryAdcVoltage * (BATTERY_SENSOR_MAX_VOLTAGE / ADC_REFERENCE_VOLTAGE);
  float currentAmps = currentAmpsFromSensorVoltage(currentVoltage);
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

bool recordStandaloneTelemetrySnapshot() {
  if (!ensureStandaloneLogDirectory() || serviceMode || sdTransferActive) return false;
  const String path = standaloneLogPath("telemetry", ".csv");
  File file = SD.open(path.c_str(), FILE_APPEND);
  if (!file) {
    ++sdWriteFailures;
    return false;
  }
  if (file.size() == 0) {
    file.print(F("boot_id,timestamp_us,gps_valid,gps_age_ms,gps_date_ddmmyy,gps_time_hhmmsscc,latitude,longitude,gps_satellites,gps_hdop,speed_mps,course_deg,gps_altitude_m,gps_altitude_age_ms,gps_satellites_age_ms,gps_hdop_age_ms,gps_speed_age_ms,gps_course_age_ms,control_mode,control_source,telemetry_mode,hold_position,mission_mode_requested,elrs_armed,elrs_arm_ready,elrs_rc_fresh,elrs_rc_age_ms,elrs_link_fresh,elrs_link_age_ms,elrs_link_quality,elrs_rssi_dbm,elrs_snr_db,elrs_rc_frames,elrs_radio_status,elrs_crc_errors,elrs_override_frames,elrs_crc_ok,elrs_crc_bad,elrs_short,elrs_target_reject,elrs_channel_reject,ch1_us,ch2_us,ch3_us,ch4_us,ch5_us,ch6_us,ch7_us,ch8_us,ch9_us,ch10_us,ch11_us,ch12_us,ch13_us,ch14_us,ch15_us,ch16_us,drive_rear,left,right,target_rear,target_left,target_right,pwm_rear_signed,pwm_left_signed,pwm_right_signed,motors_enabled,battery_v,current_a,battery_valid,current_valid,power_sample_age_ms,range_valid,range_mm,range_age_ms,water_temp_valid,water_temp_c,water_temp_age_ms,aux_sensor_sample_age_ms,imu_valid,ax_g,ay_g,az_g,gx_dps,gy_dps,gz_dps,imu_temp_c,tds_v,turbidity_probe_v,"));
    file.println(F("mag_available,mag_calibrated,mag_sample_valid,mag_sample_age_ms,mag_x_raw,mag_y_raw,mag_z_raw,mag_status,mag_samples_ok,mag_not_ready,mag_overflows,mag_i2c_errors,tilt_heading_deg,mission_heading_valid,mission_heading_age_ms,mission_heading_deg,mission_active,mission_waypoint_index,mission_waypoint_count,mission_checkpoint_active,mission_returning_home,mission_start_locked,mission_start_block_code,mission_run_status,mission_id,manual_assist_requested,manual_assist_heading_available,manual_assist_target_deg,mixer_input_lateral,mixer_input_thrust,mixer_input_yaw,mixer_input_tx12,science_state,calibration_active,calibration_test_active,sd_mount_failures,sd_write_failures,audio_frames_dropped,last_failsafe"));
  }

  const uint32_t nowMs = millis();
  const bool rcFresh = elrs.rcFresh(nowMs);
  const bool linkFresh = elrs.linkFresh(nowMs);
  const bool gpsFresh = gps.location.isValid() && gps.location.age() < 2000;
  if (!standaloneFirstGpsFixRecorded && gpsFresh && gps.date.isValid() && gps.time.isValid()) {
    File manifest = SD.open((standaloneRunDirectory() + "/run.txt").c_str(), FILE_APPEND);
    if (manifest) {
      manifest.println(String("first_gps_fix_utc=") + currentScienceTimestamp());
      manifest.println(String("first_gps_fix_uptime_ms=") + String(nowMs));
      standaloneFirstGpsFixRecorded = true;
      manifest.close();
    } else {
      ++sdWriteFailures;
    }
  }
  file.print(standaloneBootId());
  file.print(',');
  printSdTimestamp(file, static_cast<uint64_t>(esp_timer_get_time()));
  file.print(',');
  file.print(gpsFresh ? 1 : 0);
  file.print(',');
  if (gps.location.isValid()) file.print(gps.location.age());
  file.print(',');
  if (gps.date.isValid()) file.print(gps.date.value());
  file.print(',');
  if (gps.time.isValid()) file.print(gps.time.value());
  file.print(',');
  if (gpsFresh) file.print(gps.location.lat(), 7);
  file.print(',');
  if (gpsFresh) file.print(gps.location.lng(), 7);
  file.print(',');
  if (gps.satellites.isValid()) file.print(gps.satellites.value());
  file.print(',');
  if (gps.hdop.isValid()) file.print(gps.hdop.hdop(), 2);
  file.print(',');
  if (gps.speed.isValid()) file.print(gps.speed.mps(), 3);
  file.print(',');
  if (gps.course.isValid()) file.print(gps.course.deg(), 2);
  file.print(',');
  if (gps.altitude.isValid()) file.print(gps.altitude.meters(), 2);
  file.print(',');
  if (gps.altitude.isValid()) file.print(gps.altitude.age());
  file.print(',');
  if (gps.satellites.isValid()) file.print(gps.satellites.age());
  file.print(',');
  if (gps.hdop.isValid()) file.print(gps.hdop.age());
  file.print(',');
  if (gps.speed.isValid()) file.print(gps.speed.age());
  file.print(',');
  if (gps.course.isValid()) file.print(gps.course.age());
  file.print(',');
  file.print(encodeControlMode(controlMode));
  file.print(',');
  file.print(controlSource == CONTROL_SOURCE_ELRS ? "ELRS" : controlSource == CONTROL_SOURCE_WIFI ? "WIFI" : "NONE");
  file.print(',');
  file.print(telemetryMode == TELEMETRY_MODE_MAVLINK ? "MAVLINK" : "WIFI");
  file.print(',');
  file.print(holdPositionEnabled ? 1 : 0);
  file.print(',');
  file.print(missionModeRequested ? 1 : 0);
  file.print(',');
  file.print(elrsArmed ? 1 : 0);
  file.print(',');
  file.print(elrsArmReady ? 1 : 0);
  file.print(',');
  file.print(rcFresh ? 1 : 0);
  file.print(',');
  file.print(elrs.lastRcMs() == 0 ? 0 : nowMs - elrs.lastRcMs());
  file.print(',');
  file.print(linkFresh ? 1 : 0);
  file.print(',');
  file.print(elrs.lastLinkMs() == 0 ? 0 : nowMs - elrs.lastLinkMs());
  file.print(',');
  file.print(elrs.linkQuality());
  file.print(',');
  file.print(elrs.rssiDbm());
  file.print(',');
  file.print(elrs.snrDb());
  file.print(',');
  file.print(elrs.validRcFrames());
  file.print(',');
  file.print(elrs.validRadioStatus());
  file.print(',');
  file.print(elrs.crcErrors());
  file.print(',');
  file.print(elrs.rcOverrideFrames());
  file.print(',');
  file.print(elrs.rcOverrideCrcOk());
  file.print(',');
  file.print(elrs.rcOverrideCrcBad());
  file.print(',');
  file.print(elrs.rcOverrideShort());
  file.print(',');
  file.print(elrs.rcOverrideTargetReject());
  file.print(',');
  file.print(elrs.rcOverrideChannelReject());
  for (uint8_t channel = 0; channel < 16; ++channel) {
    file.print(',');
    file.print(elrs.channel(channel));
  }
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    file.print(',');
    file.print(driveValues[i]);
  }
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    file.print(',');
    file.print(targetDriveValues[i]);
  }
  for (int i = 0; i < MOTOR_COUNT; ++i) {
    const int signedPwm = driveValues[i] < 0 ? -appliedMotorPwm[i] : appliedMotorPwm[i];
    file.print(',');
    file.print(signedPwm);
  }
  file.print(',');
  file.print(motorsEnabled() ? 1 : 0);
  file.print(',');
  file.print(latestBatteryVolts, 3);
  file.print(',');
  file.print(latestCurrentAmps, 3);
  file.print(',');
  file.print(isfinite(latestBatteryVolts) && latestBatteryVolts > 0.0f ? 1 : 0);
  file.print(',');
  file.print(isfinite(latestCurrentAmps) ? 1 : 0);
  file.print(',');
  file.print(nowMs - lastStandalonePowerSampleMs);
  file.print(',');
  file.print(latestRangeValid ? 1 : 0);
  file.print(',');
  if (latestRangeValid) file.print(latestRangeDistanceMm);
  file.print(',');
  if (lastRangeStreamMs != 0) file.print(nowMs - lastRangeStreamMs);
  file.print(',');
  file.print(ds18b20HasReading ? 1 : 0);
  file.print(',');
  if (ds18b20HasReading) file.print(ds18b20TemperatureC, 3);
  file.print(',');
  if (ds18b20HasReading) file.print(nowMs - lastDs18b20RequestMs);
  file.print(',');
  file.print(nowMs - lastStandaloneScienceSampleMs);
  file.print(',');
  file.print(standaloneImuValid ? 1 : 0);
  const float imuValues[] = {standaloneImuReadings.accelXG, standaloneImuReadings.accelYG,
    standaloneImuReadings.accelZG, standaloneImuReadings.gyroXDps, standaloneImuReadings.gyroYDps,
    standaloneImuReadings.gyroZDps, standaloneImuReadings.temperatureC};
  for (float value : imuValues) {
    file.print(',');
    if (standaloneImuValid) file.print(value, 4);
  }
  file.print(',');
  printCsvFloat(file, standaloneTdsVoltage, 5);
  file.print(',');
  printCsvFloat(file, standaloneTurbidityVoltage, 5);
  const bool magSampleFresh = magnetometerAvailable && lastMagnetometerSampleValid &&
    nowMs - lastMagnetometerSampleMs <= MISSION_HEADING_MAX_AGE_MS;
  file.print(',');
  file.print(magnetometerAvailable ? 1 : 0);
  file.print(',');
  file.print(magnetometerCalibrationLoaded ? 1 : 0);
  file.print(',');
  file.print(magSampleFresh ? 1 : 0);
  file.print(',');
  if (lastMagnetometerSampleValid) file.print(nowMs - lastMagnetometerSampleMs);
  file.print(',');
  if (magSampleFresh) file.print(lastMagnetometerReadings.x);
  file.print(',');
  if (magSampleFresh) file.print(lastMagnetometerReadings.y);
  file.print(',');
  if (magSampleFresh) file.print(lastMagnetometerReadings.z);
  file.print(',');
  file.print(lastMagnetometerStatus);
  file.print(',');
  file.print(magnetometerSamplesOk);
  file.print(',');
  file.print(magnetometerNotReady);
  file.print(',');
  file.print(magnetometerOverflows);
  file.print(',');
  file.print(magnetometerI2cErrors);
  float loggedHeading = NAN;
  if (magSampleFresh && magnetometerCalibrationLoaded && standaloneImuValid &&
      nowMs - lastStandaloneScienceSampleMs <= MISSION_HEADING_MAX_AGE_MS) {
    calculateTiltHeadingDegrees(lastMagnetometerReadings, standaloneImuReadings, loggedHeading);
  }
  file.print(',');
  printCsvFloat(file, loggedHeading, 2);
  const bool missionHeadingFresh = magnetometerAvailable && missionHeadingCacheValid &&
    nowMs - missionHeadingCacheUpdatedMs <= MISSION_HEADING_MAX_AGE_MS;
  file.print(',');
  file.print(missionHeadingFresh ? 1 : 0);
  file.print(',');
  if (missionHeadingCacheValid) file.print(nowMs - missionHeadingCacheUpdatedMs);
  file.print(',');
  if (missionHeadingFresh) file.print(missionHeadingCacheDegrees, 2);
  file.print(',');
  file.print(missionExecutionActive ? 1 : 0);
  file.print(',');
  if (missionExecutionActive && missionCurrentWaypoint < missionWaypointCount) file.print(missionCurrentWaypoint + 1);
  file.print(',');
  file.print(missionWaypointCount);
  file.print(',');
  file.print(missionCheckpointActive ? 1 : 0);
  file.print(',');
  file.print(missionReturningHome ? 1 : 0);
  file.print(',');
  file.print(missionStartLocked ? 1 : 0);
  file.print(',');
  file.print(missionStartBlockCode);
  file.print(',');
  printCsvQuoted(file, missionRunStatus);
  file.print(',');
  printCsvQuoted(file, missionExecutionActive ? String(runningMissionId) : String(missionId));
  file.print(',');
  file.print(manualAssistRequested ? 1 : 0);
  file.print(',');
  file.print(manualAssistHeadingAvailable ? 1 : 0);
  file.print(',');
  if (manualAssistTargetValid) file.print(manualAssistTargetHeadingDegrees, 2);
  file.print(',');
  file.print(latestMixerInputLateral);
  file.print(',');
  file.print(latestMixerInputThrust);
  file.print(',');
  file.print(latestMixerInputYaw);
  file.print(',');
  file.print(latestMixerInputTx12 ? 1 : 0);
  file.print(',');
  file.print(scienceExperimentState);
  file.print(',');
  file.print(elrsCalibrationActive ? 1 : 0);
  file.print(',');
  file.print(elrsCalibrationTestActive ? 1 : 0);
  file.print(',');
  file.print(sdMountFailures);
  file.print(',');
  file.print(sdWriteFailures);
  file.print(',');
  file.print(audioFramesDropped);
  file.print(',');
  printCsvQuoted(file, lastElrsFailsafeStatus);
  file.println();
  file.flush();
  const bool writeError = file.getWriteError();
  file.close();
  if (writeError) ++sdWriteFailures;
  return !writeError;
}

void printMissionSampleContext(File& file) {
  file.print(',');
  if (missionExecutionActive && missionCurrentWaypoint < missionWaypointCount) {
    file.print(missionCurrentWaypoint + 1);
  }
  file.print(',');
  file.print(missionExecutionActive && missionCheckpointActive ? 1 : 0);
  file.print(',');
  if (missionExecutionActive && missionCurrentWaypoint < missionWaypointCount) {
    file.print(missionWaypoints[missionCurrentWaypoint].measurementMask);
  } else {
    file.print(0);
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
  if (!isnan(currentVoltage)) updateLatestCurrentSample(currentVoltage);
  if (!isnan(batteryAdcVoltage)) {
    updateLatestBatteryVoltage(batteryAdcVoltage * (BATTERY_SENSOR_MAX_VOLTAGE / ADC_REFERENCE_VOLTAGE));
  }

  printSdTimestamp(sdScienceFile, static_cast<uint64_t>(esp_timer_get_time()));
  printMissionSampleContext(sdScienceFile);
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
  printMissionSampleContext(sdTelemetryFile);
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
  if (!sdCardMounted) return;
  if (!serviceMode && !sdTransferActive &&
      nowMs - lastStandalonePowerSampleMs >= ELRS_BATTERY_INTERVAL_MS) {
    lastStandalonePowerSampleMs = nowMs;
    updateLatestBatteryVoltage(readBatterySystemVoltage());
    readCurrentSensorAmps();
  }
  const uint32_t standaloneSensorIntervalMs = missionExecutionActive && missionCheckpointActive
    ? SD_SCIENCE_INTERVAL_MS
    : (audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : SD_STANDALONE_TELEMETRY_INTERVAL_MS);
  if (!serviceMode && !sdTransferActive && nowMs - lastStandaloneScienceSampleMs >= standaloneSensorIntervalMs) {
    lastStandaloneScienceSampleMs = nowMs;
    standaloneImuValid = readMpu6050(standaloneImuReadings);
    standaloneTdsVoltage = readAds1115AverageVoltage(ADS1115_TDS_CHANNEL, 1);
    const float turbidityAdcVoltage = readAds1115AverageVoltage(ADS1115_TURBIDITY_CHANNEL, 1);
    standaloneTurbidityVoltage = isnan(turbidityAdcVoltage)
      ? NAN : turbidityAdcVoltage / TURBIDITY_INPUT_DIVIDER_RATIO;
  }
  if (!serviceMode && !sdTransferActive &&
      nowMs - lastStandaloneTelemetryMs >= SD_STANDALONE_TELEMETRY_INTERVAL_MS) {
    lastStandaloneTelemetryMs = nowMs;
    recordStandaloneTelemetrySnapshot();
  }
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
  uint32_t sensorLogIntervalMs = audioRecordingEnabled && !missionCheckpointActive
    ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : SD_SCIENCE_INTERVAL_MS;
  uint32_t telemetryLogIntervalMs = audioRecordingEnabled && !missionCheckpointActive
    ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : SD_TELEMETRY_INTERVAL_MS;
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

bool qmc5883WriteRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC5883_I2C_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool qmc5883ReadBytes(uint8_t startReg, uint8_t* buffer, size_t length) {
  Wire.beginTransmission(QMC5883_I2C_ADDRESS);
  Wire.write(startReg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t bytesRead = Wire.requestFrom(static_cast<int>(QMC5883_I2C_ADDRESS), static_cast<int>(length), static_cast<int>(true));
  if (bytesRead != length) return false;
  for (size_t i = 0; i < length; ++i) {
    if (Wire.available() <= 0) return false;
    buffer[i] = static_cast<uint8_t>(Wire.read());
  }
  return true;
}

bool setupQmc5883() {
  Wire.beginTransmission(QMC5883_I2C_ADDRESS);
  if (Wire.endTransmission() != 0) return false;
  if (!qmc5883WriteRegister(0x0A, 0x80)) return false;  // soft reset
  delay(10);
  if (!qmc5883WriteRegister(0x0B, 0x01)) return false;  // recommended set/reset period
  // OSR=512, range=8 G, output rate=50 Hz, continuous measurement.
  return qmc5883WriteRegister(0x09, 0xD5);
}

bool readQmc5883(MagnetometerReadings& readings) {
  if (!magnetometerAvailable) return false;
  uint8_t raw[6] = {};
  uint8_t status = 0;
  if (!qmc5883ReadBytes(0x06, &status, 1) ||
      !qmc5883ReadBytes(0x00, raw, sizeof(raw))) {
    ++magnetometerI2cErrors;
    magnetometerAvailable = false;
    lastMagnetometerSampleValid = false;
    missionHeadingCacheValid = false;
    return false;
  }
  lastMagnetometerStatus = status;
  if ((status & 0x02) != 0) {
    ++magnetometerOverflows;
    return false;
  }
  if ((status & 0x01) == 0) {
    ++magnetometerNotReady;
    return false;
  }
  readings.x = static_cast<int16_t>((static_cast<uint16_t>(raw[1]) << 8) | raw[0]);
  readings.y = static_cast<int16_t>((static_cast<uint16_t>(raw[3]) << 8) | raw[2]);
  readings.z = static_cast<int16_t>((static_cast<uint16_t>(raw[5]) << 8) | raw[4]);
  lastMagnetometerReadings = readings;
  lastMagnetometerSampleMs = millis();
  lastMagnetometerSampleValid = true;
  ++magnetometerSamplesOk;
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
  const float measuredBatteryVolts = readBatterySystemVoltage();
  const bool batteryValid = updateLatestBatteryVoltage(measuredBatteryVolts);
  const float batteryVolts = batteryValid ? measuredBatteryVolts :
    (latestBatteryVoltageValid ? latestBatteryVolts : 0.0f);
  float currentAmps = readCurrentSensorAmps();
  lastStandalonePowerSampleMs = millis();
  int batteryPct = batteryValid || latestBatteryVoltageValid ? estimateBatteryPercent(batteryVolts) : 0;

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

void sendUdpElrsChannelsTelemetry() {
  ElrsChannelsPacketPayload payload = {};
  if (elrs.rcFresh(millis())) {
    for (uint8_t channel = 0; channel < 16; ++channel) {
      payload.channels[channel] = elrs.channel(channel);
    }
  }
  sendUdpPacket(PACKET_TYPE_ELRS_CHANNELS, payload);
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

uint32_t missionCrc32File(const char* path, bool& ok) {
  ok = false;
  File file = SD.open(path, FILE_READ);
  if (!file) return 0;
  uint32_t crc = 0xFFFFFFFFUL;
  while (file.available()) {
    crc ^= static_cast<uint8_t>(file.read());
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320UL & (-(static_cast<int32_t>(crc & 1))));
  }
  file.close();
  ok = true;
  return crc ^ 0xFFFFFFFFUL;
}

void handleControlCommand(String command) {
  command.trim();
  command.toUpperCase();

  if (command.length() == 0) {
    return;
  }

  const bool motorCalibrationCommand = command.startsWith("CTRL MOTOR CAL ");
  if (motorCalibrationCommand && (elrsArmed || missionModeRequested)) {
    emitLine("ERR MOTOR CAL REQUIRES DISARMED MANUAL MODE");
    return;
  }

  const bool mixerCalibrationMutation = command.startsWith("CTRL MIX POINT ") || command == "CTRL MIX SAVE";
  if (mixerCalibrationMutation && (elrsArmed || missionModeRequested)) {
    emitLine("ERR MIX CAL REQUIRES DISARMED MANUAL MODE");
    return;
  }

  const bool wifiPropulsionCommand = command.startsWith("CTRL MOTION ") ||
    command.startsWith("CTRL VECTOR ") ||
    (command.startsWith("CTRL MOTOR ") && !motorCalibrationCommand) ||
    command.startsWith("CTRL HOLD ") || command.startsWith("CTRL GOTO ");
  if (elrsCalibrationActive && (wifiPropulsionCommand || command.startsWith("CTRL MOTOR ") ||
      command.startsWith("CTRL MIX ") || command.startsWith("CTRL CONFIG MAGCAL") ||
      command.startsWith("CTRL SCIENCE ") || command.startsWith("CTRL SERVICE ") ||
      command.startsWith("CTRL MISSION "))) {
    emitLine("ERR CALIBRATION ACTIVE");
    return;
  }
  if (missionModeRequested && wifiPropulsionCommand && command != "CTRL MOTOR STOP") {
    emitLine("ERR MISSION MODE UNAVAILABLE; PROPULSION INHIBITED");
    return;
  }
  if (wifiPropulsionCommand && elrsArmed && command != "CTRL MOTOR STOP") {
    emitLine("ERR ELRS CONTROL ACTIVE");
    return;
  }
  if (wifiPropulsionCommand && command != "CTRL MOTOR STOP") {
    if (controlSource != CONTROL_SOURCE_WIFI) recordLogLine("WIFI CONTROL AUTHORITY ACQUIRED");
    controlSource = CONTROL_SOURCE_WIFI;
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

  if (command == "CTRL MISSION STOP") {
    if (missionExecutionActive) stopMissionExecution("CONTROL_STATION_STOP", false);
    else { stopAllMotors(); missionStartLocked = true; }
    emitLine("ACK CTRL MISSION STOP");
    return;
  }
  if (command == "CTRL MISSION RETURN HOME") {
    if (!missionExecutionActive || !missionMetadata.hasHome) {
      emitLine("ERR MISSION NOT RUNNING");
      return;
    }
    beginMissionReturnHome("OPERATOR_REQUEST");
    emitLine("ACK CTRL MISSION RETURN HOME");
    return;
  }

  if (command.startsWith("CTRL CONFIG MAGCAL ")) {
    float offsetX = 0.0f, offsetY = 0.0f, scaleX = 1.0f, scaleY = 1.0f;
    if (sscanf(command.c_str(), "CTRL CONFIG MAGCAL %f %f %f %f", &offsetX, &offsetY, &scaleX, &scaleY) != 4 ||
        !validMagnetometerCalibration(offsetX, offsetY, scaleX, scaleY)) {
      emitLine("ERR CONFIG MAGCAL VALUES");
      return;
    }
    if (!saveMagnetometerCalibrationToSd(offsetX, offsetY, scaleX, scaleY)) {
      emitLine(!sdCardMounted ? "ERR CONFIG SD UNAVAILABLE" : "ERR CONFIG MAGCAL SAVE");
      return;
    }
    emitLine("ACK CONFIG MAGCAL STORED SD");
    emitLine("TEL STATUS MAGCAL LOADED SD");
    return;
  }

  if (command.startsWith("CTRL MISSION FILE BEGIN ")) {
    unsigned long byteCount = 0, checksum = 0;
    if (sscanf(command.c_str(), "CTRL MISSION FILE BEGIN %lu %lx", &byteCount, &checksum) != 2 ||
        byteCount == 0 || byteCount > 20000 || !sdCardMounted) {
      emitLine(!sdCardMounted ? "ERR MISSION SD UNAVAILABLE" : "ERR MISSION FILE BAD SIZE");
      return;
    }
    if (runningMissionId[0]) { emitLine("ERR MISSION RUNNING"); return; }
    if (!SD.exists("/missions") && !SD.mkdir("/missions")) { emitLine("ERR MISSION SD WRITE"); return; }
    if (missionUploadFile) missionUploadFile.close();
    SD.remove("/missions/upload.tmp");
    missionUploadFile = SD.open("/missions/upload.tmp", FILE_WRITE);
    if (!missionUploadFile) { emitLine("ERR MISSION SD WRITE"); return; }
    missionUploadExpectedBytes = static_cast<uint32_t>(byteCount);
    missionUploadExpectedCrc = static_cast<uint32_t>(checksum);
    missionUploadReceivedBytes = 0;
    missionUploadInProgress = true;
    emitLine("ACK MISSION FILE BEGIN " + String(missionUploadExpectedBytes));
    return;
  }

  if (command.startsWith("CTRL MISSION FILE DATA ")) {
    unsigned long offset = 0;
    char encoded[481] = "";
    if (!missionUploadInProgress || !missionUploadFile ||
        sscanf(command.c_str(), "CTRL MISSION FILE DATA %lu %480s", &offset, encoded) != 2 ||
        offset != missionUploadReceivedBytes) { emitLine("ERR MISSION FILE OFFSET"); return; }
    const size_t encodedLength = strlen(encoded);
    if (encodedLength == 0 || encodedLength > 480 || (encodedLength & 1) ||
        missionUploadReceivedBytes + encodedLength / 2 > missionUploadExpectedBytes) {
      emitLine("ERR MISSION FILE DATA"); return;
    }
    for (size_t i = 0; i < encodedLength; i += 2) {
      char pair[3] = {encoded[i], encoded[i + 1], 0};
      char* end = nullptr;
      unsigned long value = strtoul(pair, &end, 16);
      if (!end || *end != '\0' || end == pair) { emitLine("ERR MISSION FILE DATA"); return; }
      if (missionUploadFile.write(static_cast<uint8_t>(value)) != 1) {
        emitLine("ERR MISSION SD WRITE"); return;
      }
    }
    missionUploadReceivedBytes += static_cast<uint32_t>(encodedLength / 2);
    emitLine("ACK MISSION FILE DATA " + String(missionUploadReceivedBytes));
    return;
  }

  if (command == "CTRL MISSION FILE COMMIT") {
    if (!missionUploadInProgress || !missionUploadFile) { emitLine("ERR MISSION FILE NOT STARTED"); return; }
    missionUploadFile.flush();
    missionUploadFile.close();
    bool crcOk = false;
    const uint32_t actualCrc = missionCrc32File("/missions/upload.tmp", crcOk);
    MissionWaypoint verifiedPoints[MAX_MISSION_WAYPOINTS] = {};
    uint8_t verifiedCount = 0;
    char verifiedId[37] = "";
    MissionMetadata verifiedMetadata;
    if (missionUploadReceivedBytes != missionUploadExpectedBytes || !crcOk || actualCrc != missionUploadExpectedCrc ||
        !readMissionFile("/missions/upload.tmp", verifiedPoints, verifiedCount, verifiedId, &verifiedMetadata)) {
      missionUploadInProgress = false;
      SD.remove("/missions/upload.tmp");
      emitLine("ERR MISSION FILE VALIDATION");
      return;
    }
    const String missionPath = currentMissionFilePath();
    const String backupPath = currentMissionFilePath(".bak");
    SD.remove(backupPath.c_str());
    const bool hadPrevious = SD.exists(missionPath.c_str());
    if (hadPrevious && !SD.rename(missionPath.c_str(), backupPath.c_str())) {
      missionUploadInProgress = false; SD.remove("/missions/upload.tmp");
      emitLine("ERR MISSION SD WRITE"); return;
    }
    if (!SD.rename("/missions/upload.tmp", missionPath.c_str()) || !persistMissionSelection(verifiedId) ||
        !loadMissionFromSd(verifiedId)) {
      SD.remove(missionPath.c_str());
      if (hadPrevious) SD.rename(backupPath.c_str(), missionPath.c_str());
      missionUploadInProgress = false;
      emitLine("ERR MISSION SD SELECT"); return;
    }
    SD.remove(backupPath.c_str());
    missionUploadInProgress = false;
    cleanupObsoleteMissionFiles();
    emitLine("ACK MISSION COMMIT " + String(missionWaypointCount));
    emitLine("TEL STATUS MISSION " + String(missionWaypointCount) + " UPLOAD SD LOADED " +
      String(missionId) + " RUNNING " + String(runningMissionId[0] ? runningMissionId : "NONE"));
    return;
  }

  if (command.startsWith("CTRL MISSION BEGIN ")) {
    char requestedId[37] = "";
    unsigned int count = 0, boundaryCount = 0, hasHome = 0, hasEnd = 0;
    if (sscanf(command.c_str(), "CTRL MISSION BEGIN %36s %u %u %u %u", requestedId, &count, &boundaryCount, &hasHome, &hasEnd) != 5 ||
        !validMissionId(requestedId) ||
        count == 0 || count > MAX_MISSION_WAYPOINTS || boundaryCount > MAX_MISSION_BOUNDARY_POINTS || hasHome > 1 || hasEnd > 1) {
      missionUploadInProgress = false;
      emitLine("ERR MISSION BAD COUNT");
      return;
    }
    stagedMissionWaypointCount = static_cast<uint8_t>(count);
    stagedMissionMetadata = MissionMetadata();
    stagedMissionBoundaryCount = static_cast<uint8_t>(boundaryCount);
    stagedMissionMetadata.boundaryCount = stagedMissionBoundaryCount;
    stagedMissionHasHome = hasHome != 0; stagedMissionHasEnd = hasEnd != 0;
    stagedMissionHomeReceived = false; stagedMissionEndReceived = false;
    stagedMissionBoundaryMask = 0;
    strlcpy(stagedMissionId, requestedId, sizeof(stagedMissionId));
    stagedMissionWaypointMask = 0;
    missionUploadInProgress = true;
    emitLine("ACK MISSION BEGIN " + String(count));
    return;
  }

  if (command.startsWith("CTRL MISSION HOME ") || command.startsWith("CTRL MISSION END ")) {
    const bool isHome = command.startsWith("CTRL MISSION HOME ");
    float latitude = 0, longitude = 0;
    if (!missionUploadInProgress || sscanf(command.c_str(), isHome ? "CTRL MISSION HOME %f %f" : "CTRL MISSION END %f %f", &latitude, &longitude) != 2 ||
        !isfinite(latitude) || !isfinite(longitude) || latitude < -90 || latitude > 90 || longitude < -180 || longitude > 180 ||
        (isHome && !stagedMissionHasHome) || (!isHome && !stagedMissionHasEnd)) { emitLine("ERR MISSION BAD LOCATION"); return; }
    if (isHome) { stagedMissionMetadata.hasHome = true; stagedMissionMetadata.home = {latitude, longitude}; stagedMissionHomeReceived = true; emitLine("ACK MISSION HOME"); }
    else { stagedMissionMetadata.hasEnd = true; stagedMissionMetadata.end = {latitude, longitude}; stagedMissionEndReceived = true; emitLine("ACK MISSION END"); }
    return;
  }

  if (command.startsWith("CTRL MISSION BOUNDARY ")) {
    unsigned int index = 0; float latitude = 0, longitude = 0;
    if (!missionUploadInProgress || sscanf(command.c_str(), "CTRL MISSION BOUNDARY %u %f %f", &index, &latitude, &longitude) != 3 ||
        index >= stagedMissionBoundaryCount || !isfinite(latitude) || !isfinite(longitude) || latitude < -90 || latitude > 90 || longitude < -180 || longitude > 180) {
      emitLine("ERR MISSION BAD BOUNDARY"); return;
    }
    stagedMissionMetadata.boundary[index] = {latitude, longitude};
    stagedMissionBoundaryMask |= (static_cast<uint64_t>(1) << index);
    emitLine("ACK MISSION BOUNDARY " + String(index)); return;
  }

  if (command.startsWith("CTRL MISSION POINT ")) {
    unsigned int index = 0;
    float latitude = 0.0f, longitude = 0.0f; unsigned int measurementMask = 0, collectData = 0, holdSeconds = 0, speedPercent = 0;
    if (!missionUploadInProgress ||
        sscanf(command.c_str(), "CTRL MISSION POINT %u %f %f %u %u %u %u", &index, &latitude, &longitude, &measurementMask, &collectData, &holdSeconds, &speedPercent) != 7 ||
        index >= stagedMissionWaypointCount || !isfinite(latitude) || !isfinite(longitude) ||
        latitude < -90.0f || latitude > 90.0f || longitude < -180.0f || longitude > 180.0f || measurementMask > 31 || collectData > 1 || holdSeconds > 3600 || speedPercent < 1 || speedPercent > 100) {
      emitLine("ERR MISSION BAD POINT");
      return;
    }
    stagedMissionWaypoints[index] = {latitude, longitude, static_cast<uint16_t>(measurementMask), collectData != 0, static_cast<uint16_t>(holdSeconds), static_cast<uint8_t>(speedPercent)};
    stagedMissionWaypointMask |= (static_cast<uint64_t>(1) << index);
    emitLine("ACK MISSION POINT " + String(index));
    return;
  }

  if (command == "CTRL MISSION COMMIT") {
    const uint64_t expectedMask = stagedMissionWaypointCount == 64
      ? UINT64_MAX
      : ((static_cast<uint64_t>(1) << stagedMissionWaypointCount) - 1);
    const uint64_t expectedBoundaryMask = stagedMissionBoundaryCount == 64 ? UINT64_MAX : (stagedMissionBoundaryCount == 0 ? 0 : ((static_cast<uint64_t>(1) << stagedMissionBoundaryCount) - 1));
    if (!missionUploadInProgress || stagedMissionWaypointCount == 0 ||
        stagedMissionWaypointMask != expectedMask || stagedMissionBoundaryMask != expectedBoundaryMask ||
        stagedMissionHomeReceived != stagedMissionHasHome || stagedMissionEndReceived != stagedMissionHasEnd) {
      missionUploadInProgress = false;
      emitLine("ERR MISSION INCOMPLETE");
      return;
    }
    if (!saveMissionToSd(stagedMissionId, stagedMissionWaypoints, stagedMissionWaypointCount, &stagedMissionMetadata)) {
      missionUploadInProgress = false;
      emitLine("ERR MISSION SD WRITE");
      return;
    }
    if (!persistMissionSelection(stagedMissionId)) {
      missionUploadInProgress = false;
      emitLine("ERR MISSION SD SELECT");
      return;
    }
    SD.remove(currentMissionFilePath(".bak").c_str());
    memcpy(missionWaypoints, stagedMissionWaypoints,
      stagedMissionWaypointCount * sizeof(MissionWaypoint));
    missionWaypointCount = stagedMissionWaypointCount;
    missionMetadata = stagedMissionMetadata;
    strlcpy(missionId, stagedMissionId, sizeof(missionId));
    missionUploadInProgress = false;
    cleanupObsoleteMissionFiles();
    emitLine("ACK MISSION COMMIT " + String(missionWaypointCount));
    emitLine("TEL STATUS MISSION " + String(missionWaypointCount) + " UPLOAD SD LOADED " +
      String(missionId) + " RUNNING " + String(runningMissionId[0] ? runningMissionId : "NONE"));
    return;
  }

  if (command == "CTRL MISSION LIST") {
    emitMissionList();
    return;
  }

  if (command.startsWith("CTRL MISSION DELETE ")) {
    char requestedId[37] = "";
    if (sscanf(command.c_str(), "CTRL MISSION DELETE %36s", requestedId) != 1 ||
        !validMissionId(requestedId)) {
      emitLine("ERR MISSION BAD ID");
      return;
    }
    if (!sdCardMounted) {
      emitLine("ERR MISSION SD UNAVAILABLE");
      return;
    }
    if (runningMissionId[0] && strcmp(runningMissionId, requestedId) == 0) {
      emitLine("ERR MISSION RUNNING");
      return;
    }
    if (!deleteMissionFromSd(requestedId)) {
      emitLine("ERR MISSION DELETE FAILED");
      return;
    }
    emitLine("ACK MISSION DELETE " + String(requestedId));
    sendStatusSnapshot();
    return;
  }

  if (command.startsWith("CTRL MISSION READ")) {
    char requestedId[37] = "";
    const bool hasId = sscanf(command.c_str(), "CTRL MISSION READ %36s", requestedId) == 1;
    MissionWaypoint points[MAX_MISSION_WAYPOINTS] = {};
    uint8_t pointCount = 0;
    char resultId[37] = "";
    MissionMetadata metadata;
    if (hasId) {
      if (!validMissionId(requestedId)) {
        emitLine("ERR MISSION BAD ID");
        return;
      }
      const String path = currentMissionFilePath();
      bool found = readMissionFile(path.c_str(), points, pointCount, resultId, &metadata) && strcmp(resultId, requestedId) == 0;
      if (!found) {
        const String backupPath = currentMissionFilePath(".bak");
        found = readMissionFile(backupPath.c_str(), points, pointCount, resultId, &metadata) && strcmp(resultId, requestedId) == 0;
      }
      if (!found) {
        emitLine("ERR MISSION NOT FOUND");
        return;
      }
    } else {
      pointCount = missionWaypointCount;
      strlcpy(resultId, missionId, sizeof(resultId));
      memcpy(points, missionWaypoints, pointCount * sizeof(MissionWaypoint));
      metadata = missionMetadata;
    }
    emitLine("ACK MISSION READ " + String(resultId) + " " + String(pointCount) + " " + String(metadata.boundaryCount) + " " + String(metadata.hasHome ? 1 : 0) + " " + String(metadata.hasEnd ? 1 : 0));
    if (metadata.hasHome) emitLine("TEL MISSION HOME " + String(metadata.home.latitude, 7) + " " + String(metadata.home.longitude, 7));
    if (metadata.hasEnd) emitLine("TEL MISSION END " + String(metadata.end.latitude, 7) + " " + String(metadata.end.longitude, 7));
    for (uint8_t i = 0; i < metadata.boundaryCount; ++i)
      emitLine("TEL MISSION BOUNDARY " + String(i) + " " + String(metadata.boundary[i].latitude, 7) + " " + String(metadata.boundary[i].longitude, 7));
    for (uint8_t i = 0; i < pointCount; ++i) {
      emitLine("TEL MISSION POINT " + String(i) + " " +
        String(points[i].latitude, 7) + " " + String(points[i].longitude, 7) + " " +
        String(points[i].measurementMask) + " " + String(points[i].collectData ? 1 : 0) + " " + String(points[i].holdSeconds) + " " + String(points[i].speedPercent));
    }
    return;
  }

  if (command == "CTRL SERVICE START") {
    if (scienceExperimentState != 0) { emitLine("ERR STOP AND SAVE MISSION FIRST"); return; }
    stopAllMotors();
    controlMode = CONTROL_MODE_IDLE;
    controlSource = CONTROL_SOURCE_NONE;
    elrsArmed = false;
    elrsArmReady = false;
    holdPositionEnabled = false;
    audioStreamEnabled = false;
    imuStreamEnabled = false;
    serviceMode = true;
    if (audioFrameQueue) xQueueReset(audioFrameQueue);
    recordLogLine("SD SERVICE MODE STARTED");
    emitLine("ACK CTRL SERVICE START");
    sendStatusSnapshot();
    return;
  }
  if (command == "CTRL SERVICE STOP") {
    serviceMode = false;
    recordLogLine("SD SERVICE MODE STOPPED");
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
    recordLogLine("SCIENCE SESSION STARTED " + sdSessionDirectory);
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
    if (manifest) {
      if (stopUtc.startsWith("00000000T")) {
        manifest.println(F("stopped_utc=UNKNOWN"));
        manifest.println("stopped_elapsed_ms=" + String(millis()));
      } else {
        manifest.println("stopped_utc=" + stopUtc);
      }
      manifest.close();
    }
    scienceExperimentState = 2;
    recordLogLine("SCIENCE SESSION STOPPED " + sdSessionDirectory);
    emitLine("ACK CTRL SCIENCE STOP " + sdSessionDirectory);
    sendStatusSnapshot();
    return;
  }
  if (command.startsWith("CTRL SCIENCE SAVE")) {
    if (sdTransferActive || sdAuxFilesPaused) { emitLine("ERR SD TRANSFER BUSY"); return; }
    String saveUtc = command.substring(17);
    saveUtc.trim();
    if (!saveSdRecorder(saveUtc)) { emitLine("ERR SCIENCE SAVE FAILED"); return; }
    recordLogLine("SCIENCE SESSION SAVED " + sdSessionDirectory);
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
    recordLogLine("CTRL STOP RECEIVED");
    stopAllMotors();
    controlMode = CONTROL_MODE_IDLE;
    controlSource = CONTROL_SOURCE_NONE;
    elrsArmed = false;
    elrsArmReady = false;
    holdPositionEnabled = false;
    emitLine("ACK CTRL STOP");
    return;
  }

  if (command.startsWith("CTRL MIX POINT ")) {
    int power = 0, axis = -1, rear = 0, frontLeft = 0, frontRight = 0;
    if (sscanf(command.c_str(), "CTRL MIX POINT %d %d %d %d %d", &power, &axis, &rear, &frontLeft, &frontRight) != 5 ||
        power < 10 || power > 100 || power % 10 != 0 || axis < 0 || axis >= MIXER_CAL_AXIS_COUNT ||
        rear < -100 || rear > 100 || frontLeft < -100 || frontLeft > 100 || frontRight < -100 || frontRight > 100) {
      emitLine("ERR MIX CAL POINT VALUES");
      return;
    }
    const uint8_t step = power / 10 - 1;
    mixerCalValues[step][axis][0] = static_cast<int8_t>(rear);
    mixerCalValues[step][axis][1] = static_cast<int8_t>(frontLeft);
    mixerCalValues[step][axis][2] = static_cast<int8_t>(frontRight);
    mixerCalPointSet[step][axis] = true;
    mixerCalProfileLoaded = false;  // A changed candidate is not active until saved and read back.
    emitLine("ACK CTRL MIX POINT " + String(power) + " " + String(axis) + " " +
      String(rear) + " " + String(frontLeft) + " " + String(frontRight));
    return;
  }

  if (command == "CTRL MIX SAVE") {
    if (!sdCardMounted) { emitLine("ERR MIX PROFILE SD UNAVAILABLE"); return; }
    if (!saveMixerCalibrationToSd()) { emitLine("ERR MIX PROFILE SAVE"); return; }
    bool crcOk = false;
    mixerCalProfileCrc = missionCrc32File("/config/propulsion.cfg", crcOk);
    emitLine(String("ACK CTRL MIX SAVE ") + String(mixerCalibrationPointCount(mixerCalPointSet)) + " " +
      String(mixerCalibrationComplete() ? 1 : 0) + " " + (crcOk ? String(mixerCalProfileCrc, HEX) : "UNKNOWN"));
    return;
  }

  if (command == "REQ MIX PROFILE") {
    if (sdCardMounted) {
      loadMixerCalibrationFromSd();  // Read the durable candidate, never echo an unsaved RAM edit.
    } else {
      memset(mixerCalPointSet, 0, sizeof(mixerCalPointSet));
      mixerCalProfileLoaded = false;
    }
    for (uint8_t step = 0; step < MIXER_CAL_POWER_STEPS; ++step) {
      for (uint8_t axis = 0; axis < MIXER_CAL_AXIS_COUNT; ++axis) {
        if (!mixerCalPointSet[step][axis]) continue;
        emitLine(String("TEL MIX POINT ") + String((step + 1) * 10) + " " + String(axis) + " " +
          String(mixerCalValues[step][axis][0]) + " " + String(mixerCalValues[step][axis][1]) + " " + String(mixerCalValues[step][axis][2]));
      }
    }
    bool crcOk = false;
    const char* profilePath = mixerCalLoadedFromBackup ? "/config/propulsion.bak" : "/config/propulsion.cfg";
    const uint32_t crc = sdCardMounted ? missionCrc32File(profilePath, crcOk) : 0;
    emitLine(String("TEL MIX PROFILE ") + String(mixerCalibrationPointCount(mixerCalPointSet)) + " " +
      String(mixerCalibrationComplete() ? 1 : 0) + " " + String(mixerCalProfileLoaded ? 1 : 0) + " " +
      (crcOk ? String(crc, HEX) : "NONE"));
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
    recordLogLine(String("MOTOR CALIBRATION ") + String(motorName) + " " + String(directionText) +
      " INVERTED " + String(directionInvertedValue) + " MAX_PWM " + String(maxPwm));
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
      controlSource = CONTROL_SOURCE_NONE;
      elrsArmed = false;
      elrsArmReady = false;
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
    controlSource = CONTROL_SOURCE_NONE;
    elrsArmed = false;
    elrsArmReady = false;
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
  sdLogBootId = esp_random();
  if (sdLogBootId == 0) sdLogBootId = static_cast<uint32_t>(millis()) ^ 0xB00F1DUL;
  manualHomePreferencesReady = manualHomePreferences.begin("buoy_home", false);
  const bool manualHomeLoaded = loadManualHomeFromPreferences();
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  magnetometerAvailable = setupQmc5883();
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  lastLedToggleMs = millis();
  setupMotors();
  stopAllMotors();
  elrs.begin();
  ads1115Available = setupAds1115();
  sdCardMounted = mountSdCard();
  magnetometerCalibrationLoaded = loadNavigationConfigFromSd();
  mixerCalProfileLoaded = loadMixerCalibrationFromSd();
  if (sdCardMounted && !loadSelectedMissionFromSd() &&
      !migrateSelectedMissionCollectionFromSd()) migrateLegacyMissionFromSd();
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

  emitLine("ESP32 buoy firmware " + firmwareVersionString());
  emitLine("ESP32 buoy firmware ready");
  emitLine("SD card: " + String(sdCardMounted ? "ready" : "unavailable"));
  emitLine(String("Mixer profile: ") + (mixerCalProfileLoaded ? (mixerCalibrationComplete() ? "loaded and active" : "loaded; incomplete, using safe default mixer") : "not configured; using safe default mixer"));
  emitLine("ADS1115: " + String(ads1115Available ? "ready" : "unavailable"));
  emitLine("MPU6050: " + String(mpu6050Available ? "ready" : "unavailable"));
  emitLine("QMC5883 magnetometer: " + String(magnetometerAvailable ? "ready at 0x0D" : "unavailable"));
  emitLine("Navigation config: " + String(magnetometerCalibrationLoaded ? "magnetometer calibration loaded from SD" : "default; no calibration file"));
  emitLine(String("TEL STATUS MANUAL_HOME_STORAGE ") +
    (manualHomeLoaded ? "LOADED NVS" : (manualHomePreferencesReady ? "NVS_READY" : "NVS_UNAVAILABLE")));
  emitLine(manualHomeLoaded
    ? String("TEL STATUS MANUAL_HOME SET ") + String(manualHome.latitude, 7) + " " + String(manualHome.longitude, 7)
    : String("TEL STATUS MANUAL_HOME UNSET"));
  emitLine("INMP441 audio: " + String(audioInputAvailable ? "ready" : "unavailable"));
  emitLine("ELRS UART2: MAVLink 460800");
  updateSdCardTelemetry();
  emitSensorDiagnostics();
  recordLogLine(String("BOOT firmware=") + firmwareVersionString() +
    " sd=" + (sdCardMounted ? "READY" : "UNAVAILABLE") +
    " ads1115=" + (ads1115Available ? "READY" : "UNAVAILABLE") +
    " manual_home=" + (manualHomeLoaded ? "NVS" : (manualHomePreferencesReady ? "UNSET" : "NVS_FAILED")) +
    " gps_uart=" + String(GPS_BAUDRATE));
}

void setupOtaService() {
  if (WiFi.status() != WL_CONNECTED || otaServiceReady) return;

  ArduinoOTA.setHostname("ESP32-BUOY");
  ArduinoOTA.setPassword(WIFI_STA_PASSWORD);
  ArduinoOTA.onStart([]() {
    otaUpdateInProgress = true;
    stopAllMotors();
    controlMode = CONTROL_MODE_IDLE;
    controlSource = CONTROL_SOURCE_NONE;
    elrsArmed = false;
    serviceMode = true;
    recordLogLine("OTA UPDATE STARTED");
    emitLine("TEL STATUS OTA UPDATE STARTED");
  });
  ArduinoOTA.onEnd([]() {
    recordLogLine("OTA UPDATE COMPLETE");
    emitLine("TEL STATUS OTA UPDATE COMPLETE");
  });
  ArduinoOTA.onError([](ota_error_t error) {
    otaUpdateInProgress = false;
    serviceMode = false;
    recordLogLine("OTA UPDATE FAILED " + String(static_cast<int>(error)));
    emitLine("TEL STATUS OTA UPDATE FAILED " + String(static_cast<int>(error)));
  });
  ArduinoOTA.begin();
  otaServiceReady = true;
  recordLogLine("OTA SERVICE READY");
  emitLine("TEL STATUS OTA READY");
}

void serviceOtaUpdate() {
  if (WiFi.status() != WL_CONNECTED) {
    otaServiceReady = false;
    return;
  }

  setupOtaService();
  if (!otaServiceReady) return;

  const bool safeToStartUpdate = !elrsArmed && controlSource != CONTROL_SOURCE_WIFI &&
    !missionExecutionActive && scienceExperimentState == 0 && !sdTransferActive &&
    !serviceMode && !elrsCalibrationActive && !motorCommandActive();
  if (otaUpdateInProgress || safeToStartUpdate) ArduinoOTA.handle();
}

void loop() {
  processElrsControl();
  if (!serviceMode && !elrsCalibrationActive) processAudioFrames();
  processTcpControl();
  if (!elrsCalibrationActive) processSdFileServer();
  serviceOtaUpdate();
  elrs.poll();
  updateMotorCommandWatchdog();
  updateMotorStallProtection(millis());
  if (serviceMode) {
    serviceElrsCalibration(millis());
    updateMissionExecution(millis());
    updateManualReturnHome(millis());
    serviceElrsTelemetry(millis());
    updateStatusLed();
    flushTcpTelemetry();
    delay(1);
    return;
  }
  processGpsSerial();
  if (!magnetometerAvailable && millis() - lastMagnetometerRetryMs >= 5000) {
    lastMagnetometerRetryMs = millis();
    magnetometerAvailable = setupQmc5883();
  }
  serviceElrsTelemetry(millis());
  updateMissionExecution(millis());
  updateManualReturnHome(millis());
  updateStatusLed();
  updateMotorRamps();
  serviceElrsCalibration(millis());

  // During the calibration run, keep the radio/control, GPS, motor safety,
  // calibration sensor reads, and TCP status path alive. Skip unrelated
  // telemetry streams and standalone SD recorder work to prioritize samples.
  if (elrsCalibrationActive) {
    flushTcpTelemetry();
    delay(1);
    return;
  }

  uint32_t nowMs = millis();
  updateDs18b20Sensor(nowMs);
  uint32_t sensorStreamIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : IMU_STREAM_INTERVAL_MS;
  uint32_t rangeIntervalMs = missionExecutionActive && missionCheckpointActive
    ? SD_SCIENCE_INTERVAL_MS
    : (audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : RANGE_STREAM_INTERVAL_MS);
  uint32_t waterTemperatureIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : WATER_TEMP_STREAM_INTERVAL_MS;
  uint32_t powerIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : POWER_STREAM_INTERVAL_MS;
  uint32_t diagnosticsIntervalMs = audioRecordingEnabled ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : 5000UL;
  if (telemetryMode == TELEMETRY_MODE_WIFI && imuStreamEnabled && controlClientConnected() &&
      (nowMs - lastImuStreamMs >= sensorStreamIntervalMs)) {
    lastImuStreamMs = nowMs;
    sendUdpImuTelemetry();
  }
  if (!sdTransferActive && nowMs - lastRangeStreamMs >= rangeIntervalMs) {
    lastRangeStreamMs = nowMs;
    latestRangeValid = readJsnSr04tDistanceMm(latestRangeDistanceMm);
    if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected()) {
      sendUdpRangeTelemetry();
    }
  }
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastWaterTempStreamMs >= waterTemperatureIntervalMs)) {
    lastWaterTempStreamMs = nowMs;
    emitWaterTemperatureTelemetry();
  }
  const uint32_t waterChemistryIntervalMs = audioRecordingEnabled
    ? AUDIO_PRIORITY_SENSOR_INTERVAL_MS : WATER_CHEMISTRY_STREAM_INTERVAL_MS;
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastWaterChemistryStreamMs >= waterChemistryIntervalMs)) {
    lastWaterChemistryStreamMs = nowMs;
    emitWaterChemistryTelemetry();
  }
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastPowerStreamMs >= powerIntervalMs)) {
    lastPowerStreamMs = nowMs;
    sendUdpPowerTelemetry();
  }
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastStateStreamMs >= STATE_STREAM_INTERVAL_MS)) {
    lastStateStreamMs = nowMs;
    sendUdpStateTelemetry();
  }
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastElrsChannelStreamMs >= ELRS_CHANNEL_STREAM_INTERVAL_MS)) {
    lastElrsChannelStreamMs = nowMs;
    sendUdpElrsChannelsTelemetry();
  }
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastMotorOutputStreamMs >= MOTOR_OUTPUT_STREAM_INTERVAL_MS)) {
    lastMotorOutputStreamMs = nowMs;
    sendMotorOutputTelemetry();
  }
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastGpsStreamMs >= GPS_STREAM_INTERVAL_MS)) {
    lastGpsStreamMs = nowMs;
    sendUdpGpsTelemetry();
  }
  if (telemetryMode == TELEMETRY_MODE_WIFI && controlClientConnected() &&
      (nowMs - lastMagnetometerStreamMs >= MAGNETOMETER_STREAM_INTERVAL_MS)) {
    lastMagnetometerStreamMs = nowMs;
    emitMagnetometerTelemetry();
  }
  serviceSdRecorder(nowMs);
  processAudioFrames();

  if (telemetryMode == TELEMETRY_MODE_WIFI && nowMs - lastSerialSensorTestMs >= diagnosticsIntervalMs) {
    lastSerialSensorTestMs = nowMs;
    emitSensorDiagnostics();
  }
  flushTcpTelemetry();
}


