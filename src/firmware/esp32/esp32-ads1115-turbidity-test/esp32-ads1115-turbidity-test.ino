#include <Arduino.h>
#include <Wire.h>

constexpr int I2C_SDA_PIN = 23;
constexpr int I2C_SCL_PIN = 27;

constexpr uint8_t ADS1115_ADDRESS = 0x48;
constexpr uint8_t TDS_CHANNEL = 2;
constexpr uint8_t TURBIDITY_CHANNEL = 3;
constexpr float ADS1115_FULL_SCALE_VOLTS = 4.096f;
constexpr float TURBIDITY_DIVIDER_RATIO = 0.60f;
constexpr int TDS_SAMPLES_PER_READING = 31;
constexpr int TURBIDITY_SAMPLES_PER_READING = 16;
constexpr uint32_t REPORT_INTERVAL_MS = 800;

// Until the DS18B20 is included in this isolated test, the manufacturer's
// temperature-compensation equation uses an explicit 25 C assumption.
constexpr float TDS_TEST_TEMPERATURE_C = 25.0f;

constexpr uint8_t ADS1115_REG_CONVERSION = 0x00;
constexpr uint8_t ADS1115_REG_CONFIG = 0x01;

uint32_t lastReportMs = 0;
uint32_t failedReports = 0;
float minimumTdsVoltage = NAN;
float maximumTdsVoltage = NAN;
float minimumTurbiditySensorVoltage = NAN;
float maximumTurbiditySensorVoltage = NAN;

bool ads1115Present() {
  Wire.beginTransmission(ADS1115_ADDRESS);
  return Wire.endTransmission() == 0;
}

bool readAds1115SingleEnded(uint8_t channel, int16_t& rawValue) {
  if (channel > 3) {
    return false;
  }

  // Single-shot conversion, single-ended A0-A3, +/-4.096 V PGA,
  // 860 samples/second, comparator disabled.
  const uint16_t config =
    0x8000 |
    ((0x04 + channel) << 12) |
    0x0200 |
    0x0100 |
    0x00E0 |
    0x0003;

  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(ADS1115_REG_CONFIG);
  Wire.write(static_cast<uint8_t>(config >> 8));
  Wire.write(static_cast<uint8_t>(config & 0xFF));
  if (Wire.endTransmission() != 0) {
    return false;
  }

  delayMicroseconds(1400);

  Wire.beginTransmission(ADS1115_ADDRESS);
  Wire.write(ADS1115_REG_CONVERSION);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  if (Wire.requestFrom(static_cast<int>(ADS1115_ADDRESS), 2, 1) != 2) {
    return false;
  }

  rawValue = static_cast<int16_t>((Wire.read() << 8) | Wire.read());
  return true;
}

float rawToVoltage(int32_t rawValue) {
  return rawValue * (ADS1115_FULL_SCALE_VOLTS / 32768.0f);
}

bool readAverage(uint8_t channel, int sampleCount, int32_t& averageRaw, float& adcVoltage) {
  int64_t rawTotal = 0;
  int validSamples = 0;

  for (int i = 0; i < sampleCount; ++i) {
    int16_t raw = 0;
    if (readAds1115SingleEnded(channel, raw)) {
      rawTotal += raw;
      ++validSamples;
    }
  }

  if (validSamples == 0) {
    return false;
  }

  averageRaw = static_cast<int32_t>(rawTotal / validSamples);
  adcVoltage = rawToVoltage(averageRaw);
  return true;
}

bool readTdsMedian(int32_t& medianRaw, float& tdsVoltage) {
  int16_t samples[TDS_SAMPLES_PER_READING];

  for (int i = 0; i < TDS_SAMPLES_PER_READING; ++i) {
    if (!readAds1115SingleEnded(TDS_CHANNEL, samples[i])) {
      return false;
    }
  }

  for (int i = 1; i < TDS_SAMPLES_PER_READING; ++i) {
    const int16_t value = samples[i];
    int j = i - 1;
    while (j >= 0 && samples[j] > value) {
      samples[j + 1] = samples[j];
      --j;
    }
    samples[j + 1] = value;
  }

  medianRaw = samples[TDS_SAMPLES_PER_READING / 2];
  tdsVoltage = rawToVoltage(medianRaw);
  return true;
}

float estimateTdsPpm(float sensorVoltage, float waterTemperatureC) {
  const float compensationCoefficient = 1.0f + 0.02f * (waterTemperatureC - 25.0f);
  const float compensatedVoltage = sensorVoltage / compensationCoefficient;
  const float voltageSquared = compensatedVoltage * compensatedVoltage;
  const float ppm =
    (133.42f * voltageSquared * compensatedVoltage
     - 255.86f * voltageSquared
     + 857.39f * compensatedVoltage) * 0.5f;
  return ppm < 0.0f ? 0.0f : ppm;
}

const char* tdsStatus(bool readOk, int32_t raw, float voltage) {
  if (!readOk) return "READ_ERROR";
  if (voltage < -0.01f) return "CHECK_GROUND";
  if (voltage > 2.35f) return "ABOVE_TDS_RATED_OUTPUT";
  if (raw == 0) return "ZERO_OR_DISCONNECTED";
  return "OK_UNCALIBRATED";
}

const char* turbidityStatus(bool readOk, int32_t raw, float adcVoltage) {
  if (!readOk) return "READ_ERROR";
  if (adcVoltage < -0.01f) return "CHECK_GROUND";
  if (adcVoltage > 3.25f) return "INPUT_NEAR_3V3_LIMIT";
  if (raw == 0) return "ZERO_OR_DISCONNECTED";
  return "OK_UNCALIBRATED";
}

void printFloatOrBlank(float value, int digits) {
  if (!isnan(value)) {
    Serial.print(value, digits);
  }
}

void printStartupInformation() {
  Serial.println();
  Serial.println("ESP32 ADS1115 WATER-QUALITY SENSOR TEST");
  Serial.println("I2C: SDA=GPIO23 SCL=GPIO27");
  Serial.println("ADS1115: address=0x48 range=+/-4.096V, powered from 3.3V");
  Serial.println("TDS: KS0429 powered from 3.3V, OUT directly to ADS1115 A2, common GND");
  Serial.println("Turbidity: KS0414 powered from 5V, OUT through divider to ADS1115 A3");
  Serial.println("Turbidity divider: sensor OUT -> 10k -> A3; A3 -> 15k -> GND");
  Serial.println("TDS ppm uses the Keyestudio equation with an assumed 25.0 C temperature.");
  Serial.println("Voltage is measured; ppm and turbidity units require wet calibration.");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  printStartupInformation();

  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(100000);

  if (ads1115Present()) {
    Serial.println("PASS: ADS1115 acknowledged at I2C address 0x48.");
  } else {
    Serial.println("FAIL: No ADS1115 response at 0x48.");
    Serial.println("Check 3.3V, GND, SDA GPIO23, SCL GPIO27, and ADDR to GND.");
  }

  Serial.println("timestamp_ms,tds_raw_median,tds_voltage,tds_ppm_estimate_25c,tds_min_v,tds_max_v,tds_status,turbidity_raw_average,a3_voltage,turbidity_sensor_voltage,turbidity_min_v,turbidity_max_v,turbidity_status");
}

void loop() {
  const uint32_t nowMs = millis();
  if (nowMs - lastReportMs < REPORT_INTERVAL_MS) {
    return;
  }
  lastReportMs = nowMs;

  int32_t tdsRaw = 0;
  float tdsVoltage = NAN;
  const bool tdsReadOk = readTdsMedian(tdsRaw, tdsVoltage);
  const float tdsPpm = tdsReadOk
    ? estimateTdsPpm(tdsVoltage, TDS_TEST_TEMPERATURE_C)
    : NAN;

  int32_t turbidityRaw = 0;
  float turbidityAdcVoltage = NAN;
  const bool turbidityReadOk = readAverage(
    TURBIDITY_CHANNEL,
    TURBIDITY_SAMPLES_PER_READING,
    turbidityRaw,
    turbidityAdcVoltage);
  const float turbiditySensorVoltage = turbidityReadOk
    ? turbidityAdcVoltage / TURBIDITY_DIVIDER_RATIO
    : NAN;

  if (!tdsReadOk || !turbidityReadOk) {
    ++failedReports;
    if (failedReports % 8 == 0) {
      Serial.println("# Repeated read error: check ADS1115 power, address and I2C wiring.");
    }
  }

  if (tdsReadOk) {
    if (isnan(minimumTdsVoltage) || tdsVoltage < minimumTdsVoltage) minimumTdsVoltage = tdsVoltage;
    if (isnan(maximumTdsVoltage) || tdsVoltage > maximumTdsVoltage) maximumTdsVoltage = tdsVoltage;
  }
  if (turbidityReadOk) {
    if (isnan(minimumTurbiditySensorVoltage) || turbiditySensorVoltage < minimumTurbiditySensorVoltage) {
      minimumTurbiditySensorVoltage = turbiditySensorVoltage;
    }
    if (isnan(maximumTurbiditySensorVoltage) || turbiditySensorVoltage > maximumTurbiditySensorVoltage) {
      maximumTurbiditySensorVoltage = turbiditySensorVoltage;
    }
  }

  Serial.print(nowMs);
  Serial.print(',');
  if (tdsReadOk) Serial.print(tdsRaw);
  Serial.print(',');
  printFloatOrBlank(tdsVoltage, 4);
  Serial.print(',');
  printFloatOrBlank(tdsPpm, 1);
  Serial.print(',');
  printFloatOrBlank(minimumTdsVoltage, 4);
  Serial.print(',');
  printFloatOrBlank(maximumTdsVoltage, 4);
  Serial.print(',');
  Serial.print(tdsStatus(tdsReadOk, tdsRaw, tdsVoltage));
  Serial.print(',');
  if (turbidityReadOk) Serial.print(turbidityRaw);
  Serial.print(',');
  printFloatOrBlank(turbidityAdcVoltage, 4);
  Serial.print(',');
  printFloatOrBlank(turbiditySensorVoltage, 4);
  Serial.print(',');
  printFloatOrBlank(minimumTurbiditySensorVoltage, 4);
  Serial.print(',');
  printFloatOrBlank(maximumTurbiditySensorVoltage, 4);
  Serial.print(',');
  Serial.println(turbidityStatus(turbidityReadOk, turbidityRaw, turbidityAdcVoltage));
}
