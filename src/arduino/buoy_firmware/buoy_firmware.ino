struct MotorChannel {
  int speedPin;
  int directionPin1;
  int directionPin2;
};

// Pin map matched to the current Arduino Nano wiring.
MotorChannel motors[] = {
  {3, 8, 4},
  {5, 7, 9},
  {6, 11, 12},
};

const char* motorNames[] = {"Rear", "Front Left", "Front Right"};
int driveValues[] = {0, 0, 0};
bool motorsEnabled = false;

const int MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);
const int TEST_DRIVE_VALUE = 180;

// Motor axes for thrust calculation
const float rear_axis[2] = {0.0, 1.0};
const float front_left_axis[2] = {-0.86602540378, -0.5}; // -sqrt(3)/2, -0.5
const float front_right_axis[2] = {0.86602540378, -0.5}; // sqrt(3)/2, -0.5
const float* motor_axes[3] = {rear_axis, front_left_axis, front_right_axis};

float clamp(float val, float minv, float maxv) {
  return max(minv, min(maxv, val));
}

void computeMotorThrusts(float turn, float thrust, int* thrusts) {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    float motor_thrust = (2.0 / 3.0) * (turn * motor_axes[i][0] + thrust * motor_axes[i][1]);
    motor_thrust = max(0.0, motor_thrust);  // No negative thrusts for diode simulation
    motor_thrust = clamp(motor_thrust, 0.0, 1.0);
    thrusts[i] = round(motor_thrust * 180.0);
  }
}

void applyMotorOutput(const MotorChannel& motor, int driveValue) {
  int pwmValue = constrain(driveValue, 0, 180);  // Only positive for diode simulation

  if (pwmValue > 0) {
    analogWrite(motor.speedPin, pwmValue);
    digitalWrite(motor.directionPin1, HIGH);
    digitalWrite(motor.directionPin2, LOW);
  } else {
    analogWrite(motor.speedPin, 0);
    digitalWrite(motor.directionPin1, LOW);
    digitalWrite(motor.directionPin2, LOW);
  }
}

void setMotorDrive(int motorIndex, int driveValue) {
  if (motorIndex < 0 || motorIndex >= MOTOR_COUNT) {
    return;
  }

  driveValues[motorIndex] = constrain(driveValue, 0, 180);
  applyMotorOutput(motors[motorIndex], driveValues[motorIndex]);
}

void stopAllMotors() {
  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, 0);
  }
}

void handleBluetoothCommand() {
  while (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    if (command.length() == 0) {
      return;
    }

    command.toLowerCase();
    Serial.print("Bluetooth RX: ");
    Serial.println(command);

    if (command == "banana") {
      motorsEnabled = !motorsEnabled;

      if (motorsEnabled) {
        Serial.println("banana received -> motors enabled");
      } else {
        stopAllMotors();
        Serial.println("banana received -> motors stopped");
      }
    } else if (command == "stop") {
      motorsEnabled = false;
      stopAllMotors();
      Serial.println("stop received -> motors stopped");
    } else if (command == "start") {
      motorsEnabled = true;
      Serial.println("start received -> motors enabled");
    } else if (command.startsWith("vector ")) {
      if (!motorsEnabled) {
        Serial.println("Motors not enabled, ignoring VECTOR command");
        return;
      }

      int space1 = command.indexOf(' ', 7);
      int space2 = command.indexOf(' ', space1 + 1);
      if (space1 == -1 || space2 == -1) {
        Serial.println("Invalid VECTOR format");
        return;
      }

      String turnStr = command.substring(7, space1);
      String thrustStr = command.substring(space1 + 1, space2);
      int turn = turnStr.toInt();
      int thrust = thrustStr.toInt();

      float turn_f = turn / 100.0;
      float thrust_f = thrust / 100.0;

      int thrusts[3];
      computeMotorThrusts(turn_f, thrust_f, thrusts);

      for (int i = 0; i < MOTOR_COUNT; i++) {
        setMotorDrive(i, thrusts[i]);
      }

      Serial.print("VECTOR applied: x=");
      Serial.print(turn);
      Serial.print(" y=");
      Serial.print(thrust);
      Serial.print(" motors=");
      for (int i = 0; i < MOTOR_COUNT; i++) {
        Serial.print(thrusts[i]);
        if (i < MOTOR_COUNT - 1) Serial.print(",");
      }
      Serial.println();
    } else {
      Serial.print("Unknown Bluetooth command: ");
      Serial.println(command);
    }
  }
}

void setup() {
  Serial.begin(9600);
  Serial.setTimeout(100);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motors[i].speedPin, OUTPUT);
    pinMode(motors[i].directionPin1, OUTPUT);
    pinMode(motors[i].directionPin2, OUTPUT);
    applyMotorOutput(motors[i], 0);
  }

  Serial.println("Motor Control Initialized.");
  Serial.println("Bluetooth RX/TX connected on hardware Serial (RX0/TX1).");
  Serial.println("Rear motor: ENA D3, IN1 D8, IN2 D4");
  Serial.println("Front left motor: ENB D5, IN3 D7, IN4 D10");
  Serial.println("Front right motor: ENA D6, IN1 D11, IN2 D12");
  Serial.println("Motor outputs are ready.");
  Serial.println("Send 'banana' to toggle motors, 'VECTOR <turn> <thrust>' to control.");
  // Serial.println("Type banana in Serial Monitor to toggle all motors on or off.");
}

void loop() {
  handleBluetoothCommand();

  // Motors are controlled by VECTOR commands when enabled
}
