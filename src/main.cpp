#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define SERIAL_BAUD_RATE 115200

// Create a sensor object with a unique ID(12345)
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup(){
  Serial.begin(SERIAL_BAUD_RATE); // Initialize Serial comm

  // Wait a bit for connection to stabilize
  while (!Serial)
  {
    delay(10);
  }
  
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
  Serial.println("------------------------------------");
}

void loop(){
  // Get a new sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  /* * Display the results (acceleration is measured in m/s^2)
   * X: Movement Left/Right
   * Y: Movement Forward/Backward
   * Z: Movement Up/Down (Gravity pulls this, so it should read ~9.8 on flat surface)
   */
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print(" ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print(" ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print(" ");
  Serial.println("m/s^2");

  // Wait 500ms before the next read to make the text readable
  delay(500);
}