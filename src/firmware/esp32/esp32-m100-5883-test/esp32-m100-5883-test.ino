#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPSPlus.h>

// M100-5883 bench test for the buoy ESP32 wiring.
// GPS UART is receive-only: module TX -> GPIO16. Compass shares the existing
// I2C bus: SDA GPIO23, SCL GPIO27. The diagnostic page is served over Wi-Fi
// so it does not depend on USB serial (GPIO1 is used by the buoy's SD CS).
constexpr int GPS_RX_PIN = 16;
constexpr int I2C_SDA_PIN = 23;
constexpr int I2C_SCL_PIN = 27;
constexpr uint32_t GPS_BAUD = 115200;
constexpr uint8_t QMC5883_ADDRESS = 0x0D;

const char *AP_SSID = "M100-5883-Test";
const char *AP_PASSWORD = "buoytest1";

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;
WebServer server(80);
uint32_t gpsBytes = 0;
uint32_t lastGpsByteMs = 0;
bool compassFound = false;
bool compassConfigured = false;
int16_t magX = 0, magY = 0, magZ = 0;
uint8_t magStatus = 0;
uint32_t lastMagReadMs = 0;
uint32_t lastReportMs = 0;

bool writeQmc(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(QMC5883_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

bool readQmc(uint8_t reg, uint8_t *data, size_t count) {
  Wire.beginTransmission(QMC5883_ADDRESS);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  size_t received = Wire.requestFrom((int)QMC5883_ADDRESS, (int)count, (int)true);
  if (received != count) return false;
  for (size_t i = 0; i < count; ++i) data[i] = Wire.read();
  return true;
}

void readCompass() {
  if (!compassFound || millis() - lastMagReadMs < 100) return;
  lastMagReadMs = millis();
  uint8_t status = 0;
  uint8_t raw[6]{};
  if (!readQmc(0x06, &status, 1) || !readQmc(0x00, raw, sizeof(raw))) return;
  magStatus = status;
  magX = (int16_t)((uint16_t)raw[1] << 8 | raw[0]);
  magY = (int16_t)((uint16_t)raw[3] << 8 | raw[2]);
  magZ = (int16_t)((uint16_t)raw[5] << 8 | raw[4]);
}

String jsonStatus() {
  String out = "{";
  out += "\"gps_bytes\":" + String(gpsBytes);
  out += ",\"gps_age_ms\":" + String(lastGpsByteMs ? millis() - lastGpsByteMs : 0xFFFFFFFFUL);
  out += ",\"gps_fix\":" + String(gps.location.isValid() && gps.location.age() < 3000 ? "true" : "false");
  out += ",\"lat\":" + String(gps.location.isValid() ? gps.location.lat() : 0.0, 7);
  out += ",\"lon\":" + String(gps.location.isValid() ? gps.location.lng() : 0.0, 7);
  out += ",\"satellites\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0);
  out += ",\"hdop\":" + String(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, 1);
  out += ",\"speed_kmh\":" + String(gps.speed.isValid() ? gps.speed.kmph() : 0.0, 2);
  out += ",\"course_deg\":" + String(gps.course.isValid() ? gps.course.deg() : 0.0, 1);
  out += ",\"mag_found\":" + String(compassFound ? "true" : "false");
  out += ",\"mag_configured\":" + String(compassConfigured ? "true" : "false");
  out += ",\"mag_ready\":" + String((magStatus & 1) ? "true" : "false");
  out += ",\"mag_overflow\":" + String((magStatus & 2) ? "true" : "false");
  out += ",\"mag_x\":" + String(magX) + ",\"mag_y\":" + String(magY) + ",\"mag_z\":" + String(magZ);
  out += "}";
  return out;
}

const char PAGE[] PROGMEM = R"HTML(<!doctype html><html><head><meta name="viewport" content="width=device-width,initial-scale=1"><title>M100-5883 test</title><style>body{font:16px system-ui;background:#101820;color:#e8f0f4;margin:24px auto;padding:0 16px;max-width:680px}h1{font-size:1.4rem}.card{background:#1c2b35;border:1px solid #38505e;border-radius:12px;padding:16px;margin:12px 0}.row{display:flex;justify-content:space-between;gap:16px;border-bottom:1px solid #344650;padding:7px 0}.row:last-child{border:0}.v{font-family:monospace;text-align:right}.hint{color:#a9bec9;font-size:.9rem}</style></head><body><h1>HGLRC M100-5883 bench test</h1><div class="card"><h2>GPS</h2><div id="gps"></div></div><div class="card"><h2>QMC5883 compass</h2><div id="mag"></div><p class="hint">Rotate the module slowly and check whether X/Y/Z change.</p></div><p class="hint">Refreshes once per second. GPS needs a clear outdoor sky view; first fix may take a minute.</p><script>function rows(items){return items.map(x=>'<div class="row"><span>'+x[0]+'</span><span class="v">'+x[1]+'</span></div>').join('')}async function tick(){try{let d=await(await fetch('/status')).json();document.querySelector('#gps').innerHTML=rows([['UART bytes',d.gps_bytes],['Last byte age',d.gps_bytes?d.gps_age_ms+' ms':'no data'],['Valid fix',d.gps_fix],['Latitude',d.gps_fix?d.lat:'—'],['Longitude',d.gps_fix?d.lon:'—'],['Satellites',d.satellites],['HDOP',d.hdop],['Speed',d.speed_kmh+' km/h'],['Course over ground',d.course_deg+'°']]);document.querySelector('#mag').innerHTML=rows([['I²C device at 0x0D',d.mag_found],['QMC config write',d.mag_configured],['Data ready',d.mag_ready],['Overflow',d.mag_overflow],['Raw X / Y / Z',d.mag_x+' / '+d.mag_y+' / '+d.mag_z]])}catch(e){}}tick();setInterval(tick,1000)</script></body></html>)HTML";

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println();
  Serial.println("M100-5883 diagnostic boot");
  Serial.println("GPS UART: GPIO16 RX at 115200 baud; I2C: SDA 23 / SCL 27");

  pinMode(18, OUTPUT); digitalWrite(18, LOW);
  pinMode(19, OUTPUT); digitalWrite(19, LOW);
  pinMode(21, OUTPUT); digitalWrite(21, LOW);
  pinMode(22, OUTPUT); digitalWrite(22, LOW);
  pinMode(32, OUTPUT); digitalWrite(32, LOW);
  pinMode(33, OUTPUT); digitalWrite(33, LOW);

  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, -1);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);
  Serial.print("I2C scan:");
  uint8_t foundDevices = 0;
  for (uint8_t address = 0x08; address <= 0x77; ++address) {
    Wire.beginTransmission(address);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" 0x%02X", address);
      ++foundDevices;
    }
  }
  if (!foundDevices) Serial.print(" no devices found");
  Serial.println();
  Wire.beginTransmission(QMC5883_ADDRESS);
  compassFound = Wire.endTransmission() == 0;
  if (compassFound) {
    writeQmc(0x0A, 0x80); // soft reset
    delay(10);
    writeQmc(0x0B, 0x01); // recommended set/reset period
    compassConfigured = writeQmc(0x09, 0xD5); // 512 OSR, 8 G, 50 Hz, continuous
  }
  Serial.printf("QMC5883 I2C address 0x0D: %s; configuration write: %s\n",
                compassFound ? "ACK" : "no response",
                compassConfigured ? "OK" : "not configured");

  WiFi.mode(WIFI_AP);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE); });
  server.on("/status", HTTP_GET, []() { server.send(200, "application/json", jsonStatus()); });
  server.begin();
  Serial.printf("Optional Wi-Fi page: SSID %s, http://192.168.4.1\n", AP_SSID);
}

void loop() {
  while (gpsSerial.available()) {
    char c = (char)gpsSerial.read();
    ++gpsBytes;
    lastGpsByteMs = millis();
    gps.encode(c);
  }
  readCompass();
  server.handleClient();
  if (millis() - lastReportMs >= 1000) {
    lastReportMs = millis();
    Serial.printf("GPS bytes=%lu age_ms=%lu fix=%s sats=%lu HDOP=%.1f lat=%.7f lon=%.7f speed_kmh=%.2f course_deg=%.1f | MAG found=%s ready=%s overflow=%s X=%d Y=%d Z=%d\n",
                  (unsigned long)gpsBytes,
                  (unsigned long)(lastGpsByteMs ? millis() - lastGpsByteMs : 0xFFFFFFFFUL),
                  gps.location.isValid() && gps.location.age() < 3000 ? "yes" : "no",
                  (unsigned long)(gps.satellites.isValid() ? gps.satellites.value() : 0),
                  gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
                  gps.location.isValid() ? gps.location.lat() : 0.0,
                  gps.location.isValid() ? gps.location.lng() : 0.0,
                  gps.speed.isValid() ? gps.speed.kmph() : 0.0,
                  gps.course.isValid() ? gps.course.deg() : 0.0,
                  compassFound ? "yes" : "no",
                  (magStatus & 1) ? "yes" : "no",
                  (magStatus & 2) ? "yes" : "no",
                  magX, magY, magZ);
  }
  delay(1);
}
