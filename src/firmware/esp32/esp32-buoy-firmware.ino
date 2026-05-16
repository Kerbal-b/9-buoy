#include <TinyGPSPlus.h>

// ---------------------------------------------------------------------------
// Configuration / Protocol Constants
// ---------------------------------------------------------------------------

// Motor pins (grouped for easier harness routing)
const int REAR_PWM_PIN = 18;
const int REAR_DIR_PIN = 19;
const int FRONT_LEFT_PWM_PIN = 21;
const int FRONT_LEFT_DIR_PIN = 22;
const int FRONT_RIGHT_PWM_PIN = 23;
const int FRONT_RIGHT_DIR_PIN = 27;

const int CURRENT_SENSOR_PIN = 34;   // ADC1 input-only
const int BATTERY_VOLTAGE_SENSOR_PIN = 35;  // ADC1 input-only

// UART roles
// UART0: USB programming/debug (Serial)
// UART1: GPS
// UART2: control link (BT serial module / external serial bridge)
const int GPS_RX_PIN = 16;
const int GPS_TX_PIN = 17;
const int CTRL_RX_PIN = 25;
const int CTRL_TX_PIN = 26;

const long SERIAL_BAUDRATE = 115200;
const long CONTROL_BAUDRATE = 9600;
const long GPS_BAUDRATE = 9600;
const char* const FIRMWARE_BANNER = "ESP32_FW_2026_05_16";

const float ADC_REFERENCE_VOLTAGE = 3.3;
const float ADC_COUNTS = 4095.0;
const float ACS712_SENSITIVITY_VOLTS_PER_AMP = 0.066;  // 30A module
const float BATTERY_SENSOR_MAX_VOLTAGE = 25.0;  // 0-25V module scaled to ADC input
const float BATTERY_FULL_VOLTAGE = 12.6;
const float BATTERY_EMPTY_VOLTAGE = 9.6;
const int CURRENT_SENSOR_SAMPLE_COUNT = 32;
const int BATTERY_VOLTAGE_SAMPLE_COUNT = 16;
const int CURRENT_SENSOR_ZERO_CALIBRATION_SAMPLES = 200;
const float CURRENT_SENSOR_NOISE_FLOOR_AMPS = 0.15;

const int CONTROL_LINE_BUFFER_SIZE = 64;
const int MOTOR_OUTPUT_LIMIT = 255;
const int VECTOR_SCALE = 100;

const int PWM_FREQ_HZ = 20000;
const int PWM_RES_BITS = 8;
const int REAR_PWM_CHANNEL = 0;
const int FRONT_LEFT_PWM_CHANNEL = 1;
const int FRONT_RIGHT_PWM_CHANNEL = 2;

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------

struct MotorChannel {
  int speedPin;
  int directionPin;
  int pwmChannel;
};

enum ControlMode {
  CONTROL_MODE_IDLE,
  CONTROL_MODE_MANUAL,
  CONTROL_MODE_HOLD,
  CONTROL_MODE_GOTO,
};

// ---------------------------------------------------------------------------
// Hardware Configuration
// ---------------------------------------------------------------------------

MotorChannel motors[] = {
  {REAR_PWM_PIN, REAR_DIR_PIN, REAR_PWM_CHANNEL},
  {FRONT_LEFT_PWM_PIN, FRONT_LEFT_DIR_PIN, FRONT_LEFT_PWM_CHANNEL},
  {FRONT_RIGHT_PWM_PIN, FRONT_RIGHT_DIR_PIN, FRONT_RIGHT_PWM_CHANNEL},
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
HardwareSerial controlSerial(2); // UART2
TinyGPSPlus gps;

double gpsLatitude = 0.0;
double gpsLongitude = 0.0;

char controlLineBuffer[CONTROL_LINE_BUFFER_SIZE];
int controlLineLength = 0;

int driveValues[] = {0, 0, 0};
ControlMode controlMode = CONTROL_MODE_IDLE;
bool holdPositionEnabled = false;
bool targetLocationSet = false;
float targetLatitude = 0.0;
float targetLongitude = 0.0;

float currentSensorZeroVoltage = ADC_REFERENCE_VOLTAGE / 2.0;
float currentSensorZeroCounts = ADC_COUNTS / 2.0;

// ---------------------------------------------------------------------------
// Utility Helpers
// ---------------------------------------------------------------------------

float clampf(float val, float minv, float maxv) {
  return max(minv, min(maxv, val));
}

void emitLine(const String& line) {
  Serial.println(line);
  controlSerial.println(line);
}

void printControlMode(Print& stream, ControlMode mode) {
  switch (mode) {
    case CONTROL_MODE_MANUAL:
      stream.print(F("MANUAL"));
      break;
    case CONTROL_MODE_HOLD:
      stream.print(F("HOLD"));
      break;
    case CONTROL_MODE_GOTO:
      stream.print(F("GOTO"));
      break;
    case CONTROL_MODE_IDLE:
    default:
      stream.print(F("IDLE"));
      break;
  }
}

void sendStatusSnapshot() {
  controlSerial.print(F("TEL STATUS MODE "));
  printControlMode(controlSerial, controlMode);
  controlSerial.println();

  controlSerial.print(F("TEL STATUS POS "));
  if (gps.location.isValid()) {
    controlSerial.print(gps.location.lat(), 6);
    controlSerial.print(F(" "));
    controlSerial.println(gps.location.lng(), 6);
  } else {
    controlSerial.println(F("UNKNOWN UNKNOWN"));
  }

  controlSerial.print(F("TEL STATUS TARGET "));
  if (targetLocationSet) {
    controlSerial.print(targetLatitude, 6);
    controlSerial.print(F(" "));
    controlSerial.println(targetLongitude, 6);
  } else {
    controlSerial.println(F("UNKNOWN UNKNOWN"));
  }

  controlSerial.print(F("TEL STATUS HOLD "));
  controlSerial.println(holdPositionEnabled ? F("ON") : F("OFF"));

  float batteryVolts = readBatterySystemVoltage();
  float currentAmps = readCurrentSensorAmps();
  int batteryPct = estimateBatteryPercent(batteryVolts);

  controlSerial.print(F("TEL STATUS BATTERY "));
  controlSerial.print(batteryVolts, 2);
  controlSerial.print(F(" "));
  controlSerial.print(currentAmps, 2);
  controlSerial.print(F(" "));
  controlSerial.println(batteryPct);

  controlSerial.print(F("TEL STATUS CURRENT "));
  controlSerial.println(currentAmps, 2);

  controlSerial.println(F("TEL SCI WATER_TEMP UNKNOWN"));
  controlSerial.println(F("TEL SCI AIR_TEMP UNKNOWN"));
  controlSerial.println(F("TEL SCI DEPTH UNKNOWN"));
}

// ---------------------------------------------------------------------------
// Motor Control
// ---------------------------------------------------------------------------

void computeMotorThrusts(float turn, float thrust, int* thrusts) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    float motorThrust = (2.0 / 3.0) * (turn * motor_axes[i][0] + thrust * motor_axes[i][1]);
    motorThrust = clampf(motorThrust, -1.0, 1.0);
    thrusts[i] = round(motorThrust * MOTOR_OUTPUT_LIMIT);
  }
}

void applyMotorOutput(const MotorChannel& motor, int driveValue) {
  int clampedDriveValue = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  int pwmValue = abs(clampedDriveValue);

  ledcWrite(motor.pwmChannel, pwmValue);

  if (pwmValue == 0) {
    digitalWrite(motor.directionPin, LOW);
    return;
  }

  // DIR pin drives one path directly and one through inverter.
  digitalWrite(motor.directionPin, clampedDriveValue > 0 ? HIGH : LOW);
}

void setMotorDrive(int motorIndex, int driveValue) {
  if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
    return;
  }

  driveValues[motorIndex] = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  applyMotorOutput(motors[motorIndex], driveValues[motorIndex]);
}

void applyVectorCommand(int turn, int thrust) {
  float turnf = turn / static_cast<float>(VECTOR_SCALE);
  float thrustf = thrust / static_cast<float>(VECTOR_SCALE);

  int thrusts[3];
  computeMotorThrusts(turnf, thrustf, thrusts);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, thrusts[i]);
  }
}

void stopAllMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, 0);
  }
}

// ---------------------------------------------------------------------------
// Operational Sensors
// ---------------------------------------------------------------------------

float readCurrentSensorAverageCounts(int sampleCount) {
  long total = 0;
  for (int i = 0; i < sampleCount; i++) {
    total += analogRead(CURRENT_SENSOR_PIN);
  }
  return total / static_cast<float>(sampleCount);
}

float readBatterySensorAverageCounts(int sampleCount) {
  long total = 0;
  for (int i = 0; i < sampleCount; i++) {
    total += analogRead(BATTERY_VOLTAGE_SENSOR_PIN);
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

float readCurrentSensorAmps() {
  float sensorCounts = readCurrentSensorAverageCounts(CURRENT_SENSOR_SAMPLE_COUNT);
  float sensorVoltage = countsToVoltage(sensorCounts);
  float currentAmps = (sensorVoltage - currentSensorZeroVoltage) / ACS712_SENSITIVITY_VOLTS_PER_AMP;

  if (abs(currentAmps) < CURRENT_SENSOR_NOISE_FLOOR_AMPS) {
    currentAmps = 0.0;
  }

  return currentAmps;
}

// ---------------------------------------------------------------------------
// Communications / Protocol
// ---------------------------------------------------------------------------

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

void processControlSerial() {
  while (controlSerial.available() > 0) {
    char c = static_cast<char>(controlSerial.read());

    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      controlLineBuffer[controlLineLength] = '\0';
      String line(controlLineBuffer);
      controlLineLength = 0;
      handleControlCommand(line);
      continue;
    }

    if (controlLineLength < CONTROL_LINE_BUFFER_SIZE - 1) {
      controlLineBuffer[controlLineLength++] = c;
    } else {
      controlLineLength = 0;
      emitLine("ERR LINE TOO LONG");
    }
  }
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
    pinMode(motors[i].directionPin, OUTPUT);
    digitalWrite(motors[i].directionPin, LOW);

    ledcSetup(motors[i].pwmChannel, PWM_FREQ_HZ, PWM_RES_BITS);
    ledcAttachPin(motors[i].speedPin, motors[i].pwmChannel);
    ledcWrite(motors[i].pwmChannel, 0);
  }
}

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  controlSerial.begin(CONTROL_BAUDRATE, SERIAL_8N1, CTRL_RX_PIN, CTRL_TX_PIN);
  gpsSerial.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);

  analogReadResolution(12);
  analogSetPinAttenuation(CURRENT_SENSOR_PIN, ADC_11db);
  analogSetPinAttenuation(BATTERY_VOLTAGE_SENSOR_PIN, ADC_11db);

  setupMotors();
  stopAllMotors();
  calibrateCurrentSensorZero();

  Serial.println(FIRMWARE_BANNER);
  Serial.println(F("ESP32 buoy firmware ready"));
  controlSerial.println(F("ACK BOOT"));
}

void loop() {
  processControlSerial();
  processGpsSerial();
}
