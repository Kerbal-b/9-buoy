struct MotorChannel {
  int speedPin;
  int directionPin1;
  int directionPin2;
};

// Pin map matched to the current Arduino Nano wiring.
MotorChannel motors[] = {
  {3, 8, 4},
  {5, 7, 10},
  {6, 11, 12},
};

const char* motorNames[] = {"Rear", "Front Left", "Front Right"};
int driveValues[] = {0, 0, 0};

const int MOTOR_COUNT = sizeof(motors) / sizeof(motors[0]);

void applyMotorOutput(const MotorChannel& motor, int driveValue) {
  int pwmValue = abs(driveValue);

  if (driveValue > 0) {
    analogWrite(motor.speedPin, pwmValue);
    digitalWrite(motor.directionPin1, HIGH);
    digitalWrite(motor.directionPin2, LOW);
  } else if (driveValue < 0) {
    analogWrite(motor.speedPin, pwmValue);
    digitalWrite(motor.directionPin1, LOW);
    digitalWrite(motor.directionPin2, HIGH);
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

  driveValues[motorIndex] = constrain(driveValue, -255, 255);
  applyMotorOutput(motors[motorIndex], driveValues[motorIndex]);
}

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < MOTOR_COUNT; i++) {
    pinMode(motors[i].speedPin, OUTPUT);
    pinMode(motors[i].directionPin1, OUTPUT);
    pinMode(motors[i].directionPin2, OUTPUT);
    applyMotorOutput(motors[i], 0);
  }

  Serial.println("Motor Control Initialized.");
  Serial.println("Rear motor: ENA D3, IN1 D8, IN2 D4");
  Serial.println("Front left motor: ENB D5, IN3 D7, IN4 D10");
  Serial.println("Front right motor: ENA D6, IN1 D11, IN2 D12");
  Serial.println("Motor outputs are ready.");
  Serial.println("Joystick input will be handled by separate control logic.");
}

void loop() {
  // Placeholder: later code can call setMotorDrive(...) based on joystick or serial commands.
  for (int i = 0; i < MOTOR_COUNT; i++) {
    setMotorDrive(i, 0);
  }
  delay(100);
}
