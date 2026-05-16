#include <SoftwareSerial.h>
#include <AltSoftSerial.h>
#include <TinyGPSPlus.h>

// ---------------------------------------------------------------------------
// Configuration / Protocol Constants
// ---------------------------------------------------------------------------

const int BLUETOOTH_RX_PIN = 2;
const int BLUETOOTH_TX_PIN = 10;
const int CURRENT_SENSOR_PIN = A0;
const int BATTERY_VOLTAGE_SENSOR_PIN = A1;

// GPS (GT-U7) wiring - connect GT-U7 TX -> GPS_RX_PIN, GT-U7 RX -> GPS_TX_PIN
const int GPS_RX_PIN = 8;  // Arduino pin to receive GPS TX
const int GPS_TX_PIN = 9;  // Arduino pin to transmit to GPS (not required for read-only) - moved from D9 to avoid motor conflict
const long GPS_BAUDRATE = 9600;

const long SERIAL_BAUDRATE = 9600;
const long BLUETOOTH_BAUDRATE = 9600;
const char* const FIRMWARE_BANNER = "FW_PATCHED_2026_04_26";

const float ADC_REFERENCE_VOLTAGE = 5.0;
const float ADC_COUNTS = 1023.0;
const float ACS712_SENSITIVITY_VOLTS_PER_AMP = 0.066;  // 30A module
const float BATTERY_SENSOR_MAX_VOLTAGE = 25.0;  // 0-25V module scaled for 0-5V ADC input
const float BATTERY_FULL_VOLTAGE = 12.6;  // 3S lithium-ion full charge
const float BATTERY_EMPTY_VOLTAGE = 9.6;  // conservative 3S lithium-ion empty threshold
const int CURRENT_SENSOR_SAMPLE_COUNT = 32;
const int BATTERY_VOLTAGE_SAMPLE_COUNT = 16;
const int CURRENT_SENSOR_ZERO_CALIBRATION_SAMPLES = 200;
const float CURRENT_SENSOR_NOISE_FLOOR_AMPS = 0.15;

const int BLUETOOTH_LINE_BUFFER_SIZE = 48;
const int MOTOR_OUTPUT_LIMIT = 180;
const int VECTOR_SCALE = 100;

// ---------------------------------------------------------------------------
// Data Structures
// ---------------------------------------------------------------------------

struct MotorChannel {
  int speedPin;
  int directionPin;
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

// Pin map matched to the current Arduino Nano wiring.
MotorChannel motors[] = {
  {3, 12},
  {5, 7},
  {6, 11},
};

const int MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

// 120-degree thruster layout used to convert a desired motion vector into
// individual motor commands.
const float rear_axis[2] = {0.0, 1.0};
const float front_left_axis[2] = {-0.86602540378, -0.5};
const float front_right_axis[2] = {0.86602540378, -0.5};
const float* motor_axes[3] = {rear_axis, front_left_axis, front_right_axis};

// ---------------------------------------------------------------------------
// Runtime State
// ---------------------------------------------------------------------------

SoftwareSerial bluetoothSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN);
// SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN);
AltSoftSerial gpsSerial;
TinyGPSPlus gps;
double gpsLatitude = 0.0;
double gpsLongitude = 0.0;

char bluetoothLineBuffer[BLUETOOTH_LINE_BUFFER_SIZE];
int bluetoothLineLength = 0;

int driveValues[] = {0, 0, 0};
bool motorsEnabled = false;
ControlMode controlMode = CONTROL_MODE_IDLE;
bool holdPositionEnabled = false;
bool targetLocationSet = false;
float targetLatitude = 0.0;
float targetLongitude = 0.0;

float currentSensorZeroVoltage = 2.5;
float currentSensorZeroCounts = 512.0;

// ---------------------------------------------------------------------------
// Utility Helpers
// ---------------------------------------------------------------------------

float clamp(float val, float minv, float maxv) {
  return max(minv, min(maxv, val));
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

// ---------------------------------------------------------------------------
// Motor Control
// ---------------------------------------------------------------------------

void computeMotorThrusts(float turn, float thrust, int* thrusts) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    float motorThrust = (2.0 / 3.0) * (turn * motor_axes[i][0] + thrust * motor_axes[i][1]);
    motorThrust = clamp(motorThrust, -1.0, 1.0);
    thrusts[i] = round(motorThrust * MOTOR_OUTPUT_LIMIT);
  }
}

void applyMotorOutput(const MotorChannel& motor, int driveValue) {
  int clampedDriveValue = constrain(driveValue, -MOTOR_OUTPUT_LIMIT, MOTOR_OUTPUT_LIMIT);
  int pwmValue = abs(clampedDriveValue);

  analogWrite(motor.speedPin, pwmValue);

  if (pwmValue == 0) {
    digitalWrite(motor.directionPin, LOW);
    return;
  }

  // Each DIR pin feeds one motor-driver input directly and the other through
  // a 74HC14 inverter, so one Nano pin selects forward or reverse.
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

  Serial.print(F("VECTOR applied: x="));
  Serial.print(turn);
  Serial.print(F(" y="));
  Serial.print(thrust);
  Serial.print(F(" motors="));
  for (int i = 0; i < MOTOR_COUNT - 1; i++) {
    Serial.print(thrusts[i]);
    Serial.print(F(","));
  }
  Serial.println(thrusts[MOTOR_COUNT - 1]);
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
  normalized = clamp(normalized, 0.0, 1.0);
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
// Scientific Sensors
// ---------------------------------------------------------------------------

// Reserved for water temperature, depth/pressure, and other environmental
// instruments. Keep these separate from operational buoy-health sensors.

// ---------------------------------------------------------------------------
// Telemetry / Reporting
// ---------------------------------------------------------------------------

void sendStatusModeTelemetry() {
  bluetoothSerial.print(F("TEL STATUS MODE "));
  printControlMode(bluetoothSerial, controlMode);
  bluetoothSerial.println();
}

void sendStatusPositionTelemetry() {
  // Report GPS-derived position if available, otherwise fall back to UNKNOWN
  if (gps.location.isValid()) {
    // Keep 6 decimal places for compatibility with controller parsing
    bluetoothSerial.print(F("TEL STATUS POS "));
    bluetoothSerial.print(gpsLatitude, 6);
    bluetoothSerial.print(F(" "));
    bluetoothSerial.println(gpsLongitude, 6);
  } else {
    bluetoothSerial.println(F("TEL STATUS POS UNKNOWN UNKNOWN"));
  }
}

void sendStatusTargetTelemetry() {
  bluetoothSerial.print(F("TEL STATUS TARGET "));
  if (!targetLocationSet) {
    bluetoothSerial.println(F("UNKNOWN UNKNOWN"));
    return;
  }

  bluetoothSerial.print(targetLatitude, 6);
  bluetoothSerial.print(F(" "));
  bluetoothSerial.println(targetLongitude, 6);
}

void sendStatusHoldTelemetry() {
  bluetoothSerial.print(F("TEL STATUS HOLD "));
  bluetoothSerial.println(holdPositionEnabled ? F("ON") : F("OFF"));
}

void sendStatusBatteryTelemetry() {
  float batteryVoltage = readBatterySystemVoltage();
  float currentAmps = readCurrentSensorAmps();
  int batteryPercent = estimateBatteryPercent(batteryVoltage);

  // Determine charging state based on measured current (negative = charging)
  const float threshold = CURRENT_SENSOR_NOISE_FLOOR_AMPS;
  const char* charge_state = "IDLE";
  if (currentAmps <= -threshold) {
    charge_state = "CHARGING";
  } else if (currentAmps >= threshold) {
    charge_state = "DISCHARGING";
  }

  Serial.print(F("Battery voltage: "));
  Serial.print(batteryVoltage, 3);
  Serial.print(F(" V, estimated charge: "));
  Serial.print(batteryPercent);
  Serial.print(F("%"));
  Serial.print(F(", current: "));
  Serial.print(currentAmps, 3);
  Serial.print(F(" A, state: "));
  Serial.println(charge_state);

  // Send a single BATTERY telemetry line including the charge state
  bluetoothSerial.print(F("TEL STATUS BATTERY "));
  bluetoothSerial.print(batteryVoltage, 3);
  bluetoothSerial.print(F(" "));
  bluetoothSerial.print(currentAmps, 3);
  bluetoothSerial.print(F(" "));
  bluetoothSerial.print(batteryPercent);
  bluetoothSerial.print(F(" "));
  bluetoothSerial.println(charge_state);
}

void sendStatusCurrentTelemetry() {
  float currentAmps = readCurrentSensorAmps();

  Serial.print(F("Current draw: "));
  Serial.print(currentAmps, 3);
  Serial.println(F(" A"));

  bluetoothSerial.print(F("TEL STATUS CURRENT "));
  bluetoothSerial.println(currentAmps, 3);
}

void sendScienceTelemetry() {
  bluetoothSerial.println(F("TEL SCI WATER_TEMP UNKNOWN"));
  bluetoothSerial.println(F("TEL SCI AIR_TEMP UNKNOWN"));
  bluetoothSerial.println(F("TEL SCI DEPTH UNKNOWN"));
}

void sendStatusSnapshot() {
  sendStatusModeTelemetry();
  sendStatusPositionTelemetry();
  sendStatusTargetTelemetry();
  sendStatusHoldTelemetry();
  sendStatusBatteryTelemetry();
  sendStatusCurrentTelemetry();
}

void sendFullTelemetrySnapshot() {
  sendStatusSnapshot();
  sendScienceTelemetry();
}

// ---------------------------------------------------------------------------
// Communications / Bluetooth Protocol
// ---------------------------------------------------------------------------

void handleStopCommand() {
  motorsEnabled = false;
  holdPositionEnabled = false;
  controlMode = CONTROL_MODE_IDLE;
  stopAllMotors();
  Serial.println(F("CTRL STOP received -> motors stopped"));
  bluetoothSerial.println(F("ACK CTRL STOP"));
  sendStatusSnapshot();
}

void handleVectorCommand(char* payload) {
  int turn = 0;
  int thrust = 0;
  if (sscanf(payload, "%d %d", &turn, &thrust) != 2) {
    Serial.println(F("Invalid CTRL VECTOR format"));
    bluetoothSerial.println(F("ERR BAD_FORMAT"));
    return;
  }

  motorsEnabled = true;
  holdPositionEnabled = false;
  controlMode = CONTROL_MODE_MANUAL;
  applyVectorCommand(turn, thrust);

  bluetoothSerial.print(F("ACK CTRL VECTOR "));
  bluetoothSerial.print(turn);
  bluetoothSerial.print(F(" "));
  bluetoothSerial.println(thrust);
  sendStatusSnapshot();
}

void handleHoldCommand(char* payload) {
  if (strcmp(payload, "ON") == 0) {
    holdPositionEnabled = true;
    controlMode = CONTROL_MODE_HOLD;
    motorsEnabled = true;
    bluetoothSerial.println(F("ACK CTRL HOLD ON"));
    sendStatusSnapshot();
    return;
  }

  if (strcmp(payload, "OFF") == 0) {
    holdPositionEnabled = false;
    controlMode = motorsEnabled ? CONTROL_MODE_MANUAL : CONTROL_MODE_IDLE;
    bluetoothSerial.println(F("ACK CTRL HOLD OFF"));
    sendStatusSnapshot();
    return;
  }

  bluetoothSerial.println(F("ERR BAD_FORMAT"));
}

void handleGotoCommand(char* payload) {
  float latitude = 0.0;
  float longitude = 0.0;
  if (sscanf(payload, "%f %f", &latitude, &longitude) != 2) {
    bluetoothSerial.println(F("ERR BAD_FORMAT"));
    return;
  }

  targetLatitude = latitude;
  targetLongitude = longitude;
  targetLocationSet = true;
  holdPositionEnabled = false;
  motorsEnabled = true;
  controlMode = CONTROL_MODE_GOTO;

  bluetoothSerial.print(F("ACK CTRL GOTO "));
  bluetoothSerial.print(latitude, 6);
  bluetoothSerial.print(F(" "));
  bluetoothSerial.println(longitude, 6);
  sendStatusSnapshot();
}

void handleStatusRequest(char* payload) {
  if (strcmp(payload, "ALL") != 0) {
    bluetoothSerial.println(F("ERR BAD_FORMAT"));
    return;
  }

  bluetoothSerial.println(F("ACK REQ STATUS ALL"));
  sendFullTelemetrySnapshot();
}

void _processBluetoothIncomingChar(char incoming) {
  if (incoming == '\r') {
    return;
  }

  if (incoming == '\n') {
    if (bluetoothLineLength == 0) {
      return;
    }

    bluetoothLineBuffer[bluetoothLineLength] = '\0';
    processBluetoothCommand(bluetoothLineBuffer);
    bluetoothLineLength = 0;
    return;
  }

  if (bluetoothLineLength < BLUETOOTH_LINE_BUFFER_SIZE - 1) {
    bluetoothLineBuffer[bluetoothLineLength++] = incoming;
  } else {
    bluetoothLineLength = 0;
    Serial.println(F("Bluetooth command too long"));
    bluetoothSerial.println(F("ERR TOOLONG"));
  }
}

void processBluetoothCommand(char* command) {
  Serial.print(F("Bluetooth RX: "));
  Serial.println(command);

  if (strcmp(command, "PING") == 0) {
    bluetoothSerial.println(F("ACK PING"));
    sendStatusSnapshot();
    return;
  }

  if (strcmp(command, "banana") == 0) {
    motorsEnabled = !motorsEnabled;
    holdPositionEnabled = false;
    controlMode = motorsEnabled ? CONTROL_MODE_MANUAL : CONTROL_MODE_IDLE;
    if (!motorsEnabled) {
      stopAllMotors();
    }
    bluetoothSerial.println(motorsEnabled ? F("ACK CTRL HOLD OFF") : F("ACK CTRL STOP"));
    sendStatusSnapshot();
    return;
  }

  if (strcmp(command, "start") == 0) {
    motorsEnabled = true;
    holdPositionEnabled = false;
    controlMode = CONTROL_MODE_MANUAL;
    bluetoothSerial.println(F("ACK CTRL HOLD OFF"));
    sendStatusSnapshot();
    return;
  }

  if (strcmp(command, "stop") == 0) {
    handleStopCommand();
    return;
  }

  if (strcmp(command, "current") == 0 || strcmp(command, "currentraw") == 0) {
    bluetoothSerial.println(F("ACK REQ STATUS ALL"));
    sendStatusSnapshot();
    return;
  }

  if (strcmp(command, "calzero") == 0) {
    stopAllMotors();
    calibrateCurrentSensorZero();
    bluetoothSerial.println(F("ACK REQ STATUS ALL"));
    sendStatusSnapshot();
    return;
  }

  if (strncmp(command, "CTRL ", 5) == 0) {
    char* payload = command + 5;
    if (strcmp(payload, "STOP") == 0) {
      handleStopCommand();
      return;
    }

    if (strncmp(payload, "VECTOR ", 7) == 0) {
      handleVectorCommand(payload + 7);
      return;
    }

    if (strncmp(payload, "HOLD ", 5) == 0) {
      handleHoldCommand(payload + 5);
      return;
    }

    if (strncmp(payload, "GOTO ", 5) == 0) {
      handleGotoCommand(payload + 5);
      return;
    }
  }

  if (strncmp(command, "REQ STATUS ", 11) == 0) {
    handleStatusRequest(command + 11);
    return;
  }

  Serial.print(F("Unknown Bluetooth command: "));
  Serial.println(command);
  bluetoothSerial.println(F("ERR UNKNOWN"));
}

// ---------------------------------------------------------------------------
// Safety / Fault Handling
// ---------------------------------------------------------------------------

// Communication timeout, low-battery protection, overcurrent cutoffs, and
// sensor fault handling should be added here as the firmware grows.

// ---------------------------------------------------------------------------
// Arduino Lifecycle
// ---------------------------------------------------------------------------

void setup() {
  Serial.begin(SERIAL_BAUDRATE);
  Serial.setTimeout(100);
  bluetoothSerial.begin(BLUETOOTH_BAUDRATE);
  Serial.println(F("Bluetooth Serial Initialized."));
  gpsSerial.begin(GPS_BAUDRATE);
  pinMode(CURRENT_SENSOR_PIN, INPUT);
  pinMode(BATTERY_VOLTAGE_SENSOR_PIN, INPUT);
  calibrateCurrentSensorZero();

  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motors[i].speedPin, OUTPUT);
    pinMode(motors[i].directionPin, OUTPUT);
    applyMotorOutput(motors[i], 0);
  }

  Serial.println(F("Motor Control Initialized."));
  Serial.println(FIRMWARE_BANNER);
  Serial.println(F("Bluetooth RX/TX connected on SoftwareSerial (RX D2, TX D10)."));
  Serial.println(F("Rear motor: PWM D3, DIR D8, reverse from 74HC14"));
  Serial.println(F("Front left motor: PWM D5, DIR D7, reverse from 74HC14"));
  Serial.println(F("Front right motor: PWM D6, DIR D11, reverse from 74HC14"));
  Serial.println(F("ACS712 current sensor: OUT A0, VCC 5V, GND common"));
  Serial.println(F("0-25V battery voltage sensor: OUT A1, VCC 5V, GND common"));
  Serial.print(F("ACS712 zero-current counts calibrated to: "));
  Serial.println(currentSensorZeroCounts, 2);
  Serial.print(F("ACS712 zero-current voltage calibrated to: "));
  Serial.println(currentSensorZeroVoltage, 4);
  Serial.println(F("Motor outputs are ready."));
  Serial.println(F("Send 'CTRL VECTOR <turn> <thrust>', 'CTRL HOLD ON', 'CTRL GOTO <lat> <lon>', or 'REQ STATUS ALL'."));
}

unsigned long previousMillis = 0;
const long interval = 500; // 1 second interval

void loop() {
  unsigned long currentMillis = millis(); // Get current time

 // Always start by ensuring Bluetooth is the active listener
  if(!bluetoothSerial.isListening()) bluetoothSerial.listen();
  // Process all available Bluetooth characters (Burst read for responsiveness)
  while (bluetoothSerial.available()) {
    char incoming = static_cast<char>(bluetoothSerial.read());
    _processBluetoothIncomingChar(incoming);
  }
  // Check if 1 second has passed
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    // Briefly switch to GPS to read exactly one character if available
    while (gpsSerial.available()) {
      char c = gpsSerial.read();
      gps.encode(c);
      Serial.print(c);
    }
    Serial.println(' ');
  }

  // Immediately switch back to Bluetooth to maximize its listening window
  //bluetoothSerial.listen();

  // Update cached coordinates when a new valid location is available
  if (gps.location.isUpdated() && gps.location.isValid()) {
    gpsLatitude = gps.location.lat();
    gpsLongitude = gps.location.lng();
  }
  

  // Motors are controlled by VECTOR commands when enabled.
}
