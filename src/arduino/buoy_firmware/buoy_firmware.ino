// Define pins for motor control
const int ENA = 3;  // PWM speed pin
const int IN1 = 6;  // Direction pin 1
const int IN2 = 5;  // Direction pin 2
const int CONTROLLER_PIN = A0;  // Analog input from joystick or potentiometer

int motorSpeed = 0;  // Current motor speed

void setup() {
  Serial.begin(9600);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(CONTROLLER_PIN, INPUT);

  // Set direction once during startup.
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  Serial.println("Motor Control Initialized.");
  Serial.println("Controller input on A0 is now adjusting speed.");
}

void loop() {
  int controllerValue = analogRead(CONTROLLER_PIN);
  int newSpeed = map(controllerValue, 0, 1023, 0, 255);

  // Only update when the speed meaningfully changes.
  if (abs(newSpeed - motorSpeed) > 2) {
    motorSpeed = newSpeed;

    Serial.print("Controller value: ");
    Serial.print(controllerValue);
    Serial.print(" -> Motor speed: ");
    Serial.println(motorSpeed);

    analogWrite(ENA, motorSpeed);
  }

  delay(50);
}
