/*
 * PID Temperature Controller - Milestone 1
 * Temperature Sensor Test
 * 
 * Hardware:
 * - DS18B20 waterproof temperature sensor
 * - 4.7kΩ pull-up resistor between DATA and VCC
 * 
 * Wiring:
 * - RED (VCC)    → Arduino 5V
 * - BLACK (GND)  → Arduino GND
 * - YELLOW (DATA)→ Arduino Pin 2 (with 4.7kΩ pull-up to 5V)
 * 
 * Author: Gabriel Hossenboccus
 * Date: December 2024
 */

#include <OneWire.h>
#include <DallasTemperature.h>

// Pin definitions
#define ONE_WIRE_BUS 2        // DS18B20 data pin
#define LED_PIN 13            // Built-in LED for status

// Create OneWire instance
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// Variables
float temperature = 0.0;
unsigned long lastReadTime = 0;
const unsigned long READ_INTERVAL = 1000;  // Read every 1 second

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize LED
  pinMode(LED_PIN, OUTPUT);
  
  // Start temperature sensor
  sensors.begin();
  
  // Print header
  Serial.println("=================================");
  Serial.println("PID Temperature Controller");
  Serial.println("Milestone 1: Temperature Sensing");
  Serial.println("=================================");
  Serial.println();
  
  // Check if sensor is connected
  if (sensors.getDeviceCount() == 0) {
    Serial.println("ERROR: No DS18B20 sensor found!");
    Serial.println("Check wiring:");
    Serial.println("- VCC to 5V");
    Serial.println("- GND to GND");
    Serial.println("- DATA to Pin 2 with 4.7kΩ pull-up");
    while(1) {
      // Blink LED to indicate error
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      delay(200);
    }
  }
  
  Serial.print("Found ");
  Serial.print(sensors.getDeviceCount());
  Serial.println(" sensor(s)");
  Serial.println();
  Serial.println("Time(ms),Temperature(C)");  // CSV header for plotting
}

void loop() {
  unsigned long currentTime = millis();
  
  // Read temperature at specified interval
  if (currentTime - lastReadTime >= READ_INTERVAL) {
    lastReadTime = currentTime;
    
    // Request temperature reading
    sensors.requestTemperatures();
    
    // Get temperature in Celsius
    temperature = sensors.getTempCByIndex(0);
    
    // Blink LED to show activity
    digitalWrite(LED_PIN, HIGH);
    
    // Check for valid reading
    if (temperature == DEVICE_DISCONNECTED_C) {
      Serial.println("ERROR: Sensor disconnected!");
    } else {
      // Print in CSV format: timestamp, temperature
      Serial.print(currentTime);
      Serial.print(",");
      Serial.println(temperature, 2);  // 2 decimal places
    }
    
    digitalWrite(LED_PIN, LOW);
  }
}