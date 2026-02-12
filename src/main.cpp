#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "*****";
const char* password = "*****";

#define PIN_LED_RED 25 // Alarm
#define PIN_LED_GREEN 26 // Setup/Calibration 
#define PIN_LED_WHITE 27 // Heartbeat
#define PIN_BUZZER 18 // Alarm

#define SERIAL_BAUD_RATE 115200

#define P_WAVE_THRESHOLD 0.20 // Sens
#define ALARM_DURATION 3000

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // Create a sensor object with a unique ID(12345)
WebServer server(80);

// Global variable to store the previous reading
float previousMagnitude = 0;
bool firstReading = true; // Flag to handle the first reading

bool isAlarmActive = false;
unsigned long alarmOffTime = 0;

// Web page
String getHTML() {
  String html = "<!DOCTYPE html><html><head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1'>";
  html += "<style>";
  html += "body { font-family: sans-serif; text-align: center; margin-top: 50px; transition: background-color 0.5s; }";
  html += ".container { padding: 20px; }";
  html += "h1 { font-size: 50px; }";
  html += "#statusText { font-weight: bold; }";
  html += "</style>";
  
  // --- JAVASCRIPT SECTION ---
  // This script runs on your phone/browser. It asks the ESP32 for status every 500ms.

  html += "<script>";
  html += "setInterval(function() {";
  html += "  fetch('/status').then(response => response.text()).then(data => {";
  html += "    if (data == 'ALARM') {";
  html += "      document.body.style.backgroundColor = 'red';";
  html += "      document.getElementById('statusText').innerText = 'EARTHQUAKE!';";
  html += "      document.getElementById('statusText').style.color = 'white';";
  html += "    } else {";
  html += "      document.body.style.backgroundColor = 'white';";
  html += "      document.getElementById('statusText').innerText = 'SAFE';";
  html += "      document.getElementById('statusText').style.color = 'green';";
  html += "    }";
  html += "  });";
  html += "}, 500);"; // Check every 500 milliseconds
  html += "</script>";

  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h2>SEISMIC MONITOR</h2>";
  html += "<hr>";
  html += "<h1 id='statusText'>SAFE</h1>"; // This text changes via JavaScript
  html += "<p>System is monitoring P-Waves...</p>";
  html += "</div></body></html>";
  return html;
}

void handleStatus(){
  if(isAlarmActive) {
    server.send(200, "text/plain", "ALARM");
  } else {
    server.send(200, "text/plain", "SAFE");
  }
}

// Webserver Functions
void handleRoot(){
  server.send(200, "text/html", getHTML());
}

void setup(){
  Serial.begin(SERIAL_BAUD_RATE); // Initialize Serial comm

  // Pin configurations
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_WHITE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  
  // Default state: All off
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_WHITE, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  // Initialize the sensor
  if(!accel.begin()){
    Serial.println("ERROR: No ADXL345 detected! -- Check your wiring!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_16_G); // Set sensor range

  // Wifi Start
  Serial.print("Connecting Wifi...");
  Serial.print(ssid);
  WiFi.begin(ssid, password);
  while(WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("Connected! IP Address: ");
  Serial.println(WiFi.localIP());

  // Starting Webserver
  server.on("/", handleRoot); // Load the page
  server.on("/status", handleStatus); // Check status
  server.begin();

  // Starting Calibration
  for(int i = 0; i < 3; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH); delay(200);
    digitalWrite(PIN_LED_GREEN, LOW); delay(200);
  }
}

void loop(){

  server.handleClient(); // Handle Requests

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
    alarmOffTime = millis() + ALARM_DURATION;
    isAlarmActive = true;
    Serial.print("   >>> P-WAVE / VIBRATION DETECTED! <<<");
  }

  if(isAlarmActive) {
    if( millis() < alarmOffTime) {
      // TIME IS NOT UP YET -> KEEP ON
      digitalWrite(PIN_LED_RED, HIGH);
      digitalWrite(PIN_BUZZER, HIGH);
    } else {
      // TIME IS UP -> TURN OFF
      isAlarmActive = false;
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      Serial.println("--- Alarm Reset ---");
    }
  }

  previousMagnitude = currentMagnitude; // Update Previous Reading for the Next Loop
  delay(20); // Wait 500ms before the next read to make the text readable
}