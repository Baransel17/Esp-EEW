#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

#define SERIAL_BAUD_RATE 115200

/* THRESHOLD CONFIGURATION
 * Earth's gravity is ~9.8 m/s^2.
 * Any value significantly above 9.8 means external force (vibration/quake).
 * 11.0 is a sensitive threshold (light tap).
 * 15.0 would be a strong shake.
 */
#define SEISMIC_THRESHOLD 11.0

// Create a sensor object with a unique ID(12345)
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup(){
  Serial.begin(SERIAL_BAUD_RATE); // Initialize Serial comm

  // Wait a bit for connection to stabilize
  while (!Serial){ delay(10); }
  
  Serial.println("\n------------------------------------");
  Serial.println("System Starting...");
  Serial.println("Testing ADXL345 Connection...");
  Serial.println("------------------------------------");

  // Initialize the sensor
  if(!accel.begin()){
    Serial.println("ERROR: No ADXL345 detected!");
    Serial.println("Check your wiring:");
    Serial.println("  - VCC -> 3.3V");
    Serial.println("  - GND -> GND");
    Serial.println("  - SDA -> GPIO 21");
    Serial.println("  - SCL -> GPIO 22");
    while (1); // Stop here
  }

  // Set sensor range
  accel.setRange(ADXL345_RANGE_16_G);

  Serial.println("SUCCESS: ADXL345 Found!");
  Serial.println("Range set to +/- 16G");
  Serial.println("System Ready. Reading data...");
  Serial.print("Detection Threshold set to: ");
  Serial.println(SEISMIC_THRESHOLD);
  Serial.println("------------------------------------");
}

void loop(){
  // Get a new sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  // Extract Axis Data
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  // APPLY PYTHAGORAS THEOREM
  float totalMagnitude = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

  // Print Magnitude for debug
  Serial.print("Magnitude: ");
  Serial.println(totalMagnitude);

  // Earthquake Detection Logic
  // if total force is grater than threshold it'a an event!
  if(totalMagnitude > SEISMIC_THRESHOLD) {
    Serial.print("   >>> EARTHQUAKE DETECTED! <<<");
  } else {
    Serial.print("  (Normal)");
  }
  Serial.println();

  // Wait 500ms before the next read to make the text readable
  delay(500);
}