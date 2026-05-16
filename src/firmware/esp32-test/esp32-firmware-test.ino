#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

/*
  ESP32 Firmware Bring-Up + Protocol/BLE Test

  Purpose:
  - Validate ESP32 onboard BLE link with control station
  - Accept buoy control protocol messages used by the laptop app
  - Return ACK/TEL protocol lines for integration testing

  BLE profile:
  - Service UUID:        0000FFE0-0000-1000-8000-00805F9B34FB
  - RX/TX Characteristic 0000FFE1-0000-1000-8000-00805F9B34FB
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

static const char* DEVICE_NAME = "ESP32-BUOY";
static const char* SERVICE_UUID = "0000FFE0-0000-1000-8000-00805F9B34FB";
static const char* CHAR_UUID = "0000FFE1-0000-1000-8000-00805F9B34FB";

static const uint32_t SERIAL_BAUD = 115200;
static const uint32_t HEARTBEAT_MS = 1000;
static const uint32_t BLINK_MS = 250;

BLEServer* bleServer = nullptr;
BLECharacteristic* bleCharacteristic = nullptr;

bool bleClientConnected = false;
bool previousBleClientConnected = false;

uint32_t lastHeartbeatMs = 0;
uint32_t lastBlinkMs = 0;

bool ledState = false;
bool blinkEnabled = true;
int currentTurn = 0;
int currentThrust = 0;
String controlMode = "IDLE";

void emitLine(const String& line) {
  Serial.println(line);

  if (bleClientConnected && bleCharacteristic != nullptr) {
    String payload = line + "\n";
    bleCharacteristic->setValue(payload.c_str());
    bleCharacteristic->notify();
  }
}

void emitStatusSnapshot() {
  emitLine("TEL STATUS MODE " + controlMode);
  emitLine("TEL STATUS POS UNKNOWN UNKNOWN");
  emitLine("TEL STATUS TARGET UNKNOWN UNKNOWN");
  emitLine("TEL STATUS HOLD OFF");
  emitLine("TEL STATUS BATTERY UNKNOWN UNKNOWN UNKNOWN");
  emitLine("TEL STATUS CURRENT UNKNOWN");
  emitLine("TEL SCI WATER_TEMP UNKNOWN");
  emitLine("TEL SCI AIR_TEMP UNKNOWN");
  emitLine("TEL SCI DEPTH UNKNOWN");
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    bleClientConnected = true;
    (void)pServer;
  }

  void onDisconnect(BLEServer* pServer) override {
    bleClientConnected = false;
    pServer->getAdvertising()->start();
  }
};

class CharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String raw = characteristic->getValue();
    if (raw.length() == 0) {
      return;
    }

    String command;
    command.reserve(raw.length());
    for (size_t i = 0; i < raw.length(); ++i) {
      char c = raw[i];
      if (c != '\r' && c != '\n') {
        command += c;
      }
    }
    command.trim();
    if (command.length() == 0) {
      return;
    }

    command.toUpperCase();

    if (command == "PING") {
      emitLine("ACK PING");
      emitStatusSnapshot();
      return;
    }

    if (command == "REQ STATUS ALL") {
      emitLine("ACK REQ STATUS ALL");
      emitStatusSnapshot();
      return;
    }

    if (command == "CTRL STOP") {
      currentTurn = 0;
      currentThrust = 0;
      controlMode = "MANUAL";
      emitLine("ACK CTRL STOP");
      emitLine("TEL STATUS MODE " + controlMode);
      return;
    }

    if (command == "CTRL HOLD ON" || command == "CTRL HOLD OFF") {
      emitLine("ACK " + command);
      emitLine("TEL STATUS HOLD " + String(command.endsWith("ON") ? "ON" : "OFF"));
      return;
    }

    if (command.startsWith("CTRL GOTO ")) {
      emitLine("ACK " + command);
      emitLine("TEL STATUS TARGET " + command.substring(10));
      return;
    }

    if (command.startsWith("CTRL VECTOR ")) {
      int firstSpace = command.indexOf(' ', 12);
      if (firstSpace < 0) {
        emitLine("ERR BAD CTRL VECTOR");
        return;
      }

      String turnText = command.substring(12, firstSpace);
      String thrustText = command.substring(firstSpace + 1);

      int turn = turnText.toInt();
      int thrust = thrustText.toInt();

      if (turn < -100 || turn > 100 || thrust < -100 || thrust > 100) {
        emitLine("ERR RANGE CTRL VECTOR");
        return;
      }

      currentTurn = turn;
      currentThrust = thrust;
      controlMode = "MANUAL";

      emitLine("ACK CTRL VECTOR " + String(currentTurn) + " " + String(currentThrust));
      emitLine("TEL STATUS MODE " + controlMode);
      return;
    }

    emitLine("ERR UNKNOWN " + command);
  }
};

void setupBle() {
  BLEDevice::init(DEVICE_NAME);
  bleServer = BLEDevice::createServer();
  bleServer->setCallbacks(new ServerCallbacks());

  BLEService* service = bleServer->createService(SERVICE_UUID);

  bleCharacteristic = service->createCharacteristic(
    CHAR_UUID,
    BLECharacteristic::PROPERTY_READ |
      BLECharacteristic::PROPERTY_WRITE |
      BLECharacteristic::PROPERTY_NOTIFY
  );

  bleCharacteristic->addDescriptor(new BLE2902());
  bleCharacteristic->setCallbacks(new CharacteristicCallbacks());
  bleCharacteristic->setValue("READY\n");

  service->start();
  bleServer->getAdvertising()->start();
}

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Serial.begin(SERIAL_BAUD);
  delay(300);

  setupBle();

  Serial.println("ESP32 protocol/BLE test started");
  Serial.println("Device name: ESP32-BUOY");
  Serial.println("Expect control station protocol commands over BLE characteristic write");
}

void loop() {
  uint32_t nowMs = millis();

  if (blinkEnabled && (nowMs - lastBlinkMs >= BLINK_MS)) {
    lastBlinkMs = nowMs;
    ledState = !ledState;
    digitalWrite(LED_BUILTIN, ledState ? HIGH : LOW);
  }

  if (nowMs - lastHeartbeatMs >= HEARTBEAT_MS) {
    lastHeartbeatMs = nowMs;
    emitLine("TEL STATUS MODE " + controlMode);
  }

  if (!previousBleClientConnected && bleClientConnected) {
    emitLine("ACK LINK CONNECTED");
  }
  if (previousBleClientConnected && !bleClientConnected) {
    Serial.println("BLE client disconnected");
  }
  previousBleClientConnected = bleClientConnected;

  delay(5);
}
