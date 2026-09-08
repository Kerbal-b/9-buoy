#include <OneWire.h>
#include <DallasTemperature.h>

// Define the pin connected to the DS18B20 data line (D14 / GPIO14)
#define ONE_WIRE_BUS 14 

// Setup a oneWire instance to communicate with the sensor
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor template
DallasTemperature sensors(&oneWire);

void setup(void) {
  // Start serial communication at 115200 baud
  Serial.begin(115200);
  while(!Serial); // Wait for serial to initialize
  
  Serial.println("\n====================================");
  Serial.println("   DS18B20 MINIMAL DIAGNOSTIC TEST   ");
  Serial.println("====================================");
  
  // Initialize the sensor library
  sensors.begin();
  
  // Count how many sensors the microcontroller physically detects
  int deviceCount = sensors.getDeviceCount();
  Serial.print("Sensors detected on pin D14: ");
  Serial.println(deviceCount);

  // Check if the network is operating in Parasitic Power mode
  Serial.print("Parasite power mode is: "); 
  if (sensors.isParasitePowerMode()) {
    Serial.println("ON (Requires strong pull-up handling)");
  } else {
    Serial.println("OFF (External VCC detected/expected)");
  }
  Serial.println("------------------------------------");
}

void loop(void) { 
  Serial.print("Sending temperature conversion command... ");
  
  // Request temperature readings (handles timing requirements automatically)
  sensors.requestTemperatures(); 
  Serial.println("Done.");
  
  // Fetch temperature by index 0 (first discovered sensor)
  float tempC = sensors.getTempCByIndex(0);
  
  // Evaluate the raw output value
  if (tempC == DEVICE_DISCONNECTED_C) {
    Serial.println("[ERROR]: Could not read data. Sensor disconnected, missing pull-up, or timing failure.");
  } else if (tempC == 85.00) {
    Serial.println("[WARNING]: Reading is exactly 85.00°C. Power was applied, but read cycle was initiated before conversion finished.");
  } else {
    Serial.print("SUCCESS -> Temperature: ");
    Serial.print(tempC);
    Serial.println(" °C");
  }
  
  Serial.println("------------------------------------");
  delay(2000); // Wait 2 seconds before testing again
}