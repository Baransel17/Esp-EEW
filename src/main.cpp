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
#define P_WAVE_THRESHOLD 0.20

// Create a sensor object with a unique ID(12345)
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Global variable to store the previous reading
float previousMagnitude = 0;
bool firstReading = true; // Flag to handle the first reading

void setup(){
  Serial.begin(SERIAL_BAUD_RATE); // Initialize Serial comm

  // Wait a bit for connection to stabilize
  while (!Serial){ delay(10); }
  
  Serial.println("\n------------------------------------");
  Serial.println("System Starting: P-Wave Detection Mode");
  Serial.println("------------------------------------");

  // Initialize the sensor
  if(!accel.begin()){
    Serial.println("ERROR: No ADXL345 detected! -- Check your wiring!");
    while (1);
  }

  // Set sensor range
  accel.setRange(ADXL345_RANGE_16_G);

  Serial.println("System Ready.");
  Serial.print("Detection Threshold set to: ");
  Serial.println(P_WAVE_THRESHOLD);
  Serial.println("------------------------------------");

  // Initial delay to let sensor settle
  delay(500);
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
  float currentMagnitude = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));

  if (firstReading) {
    previousMagnitude = currentMagnitude;
    firstReading = false;
    return; // Skip the rest of the loop once
  }

  // Calculate DELTA (change)
  // fabs() is float absolute value (turn negatives to positives)
  float delta = fabs(currentMagnitude - previousMagnitude);
  

  // Earthquake Detection Logic
  // if total force is grater than threshold it'a an event!
  if(delta > P_WAVE_THRESHOLD) {
    Serial.print("   >>> P-WAVE / VIBRATION DETECTED! <<<");
    Serial.print(" Delta: ");
    Serial.print(delta);
    Serial.print("  | Total Mag:");
    Serial.println(currentMagnitude);
  } else {
    Serial.println(".");
  }
  
  // Update Previous Reading for the Next Loop
  previousMagnitude = currentMagnitude;

  // Wait 500ms before the next read to make the text readable
  delay(20);
}