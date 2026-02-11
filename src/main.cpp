#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>

#define PIN_LED_RED 25 // Alarm
#define PIN_LED_GREEN 26 // Setup/Calibration 
#define PIN_LED_WHITE 27 // Heartbeat
#define PIN_BUZZER 18 // Alarm

#define SERIAL_BAUD_RATE 115200

#define P_WAVE_THRESHOLD 0.20 // Sens

#define HEARTBEAT_INTERVAL 15000 // 30 sec heartbeat

// Create a sensor object with a unique ID(12345)
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// Global variable to store the previous reading
float previousMagnitude = 0;
bool firstReading = true; // Flag to handle the first reading
unsigned long lastHeartbeatTime = 0; // Timer for heartbeat

void runCalibrationSequence() {
  Serial.println(">>> CALIBRATING... DO NOT MOVE SENSOR <<<");

  // Blink green 5 times fast for calibration
  for(int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH);
    delay(200);
    digitalWrite(PIN_LED_GREEN, LOW);
    delay(200);
  }

  // Turn off green led after calibration
  digitalWrite(PIN_LED_GREEN, LOW);
  Serial.println(">>> CALIBRATION COMPLETE. SYSTEM IS NOW READY. <<<");
}

void triggerAlarm() {
  Serial.println("!!! ALARM TRIGGERED !!!");

  // Turn on alarm
  digitalWrite(PIN_LED_RED, HIGH);
  digitalWrite(PIN_BUZZER, HIGH);

  // Short warning 
  delay(500);

  // Turn off alarm
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_BUZZER, LOW);
}

void handleHeartbeat() {
  unsigned long currentMillis = millis();

  // Check if 30 seconds have passed
  if(currentMillis - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
    lastHeartbeatTime = currentMillis; // Reset timer

    Serial.println("... System Status: OK (Heartbeat) ...");

    // Blink white led once
    digitalWrite(PIN_LED_WHITE, HIGH);
    delay(200);
    digitalWrite(PIN_LED_WHITE, LOW);
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD_RATE); // Initialize Serial comm

  // Wait a bit for connection to stabilize
  while (!Serial){ delay(10); }

  // Pin configurations
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_WHITE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  
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

  runCalibrationSequence();

  lastHeartbeatTime = millis();
}

void loop(){

  handleHeartbeat();

  // Get a new sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  // Extract Axis Data
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;
  float currentMagnitude = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); // APPLY PYTHAGORAS THEOREM

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
    triggerAlarm();
  }
  
  // Update Previous Reading for the Next Loop
  previousMagnitude = currentMagnitude;

  // Wait 500ms before the next read to make the text readable
  delay(20);
}