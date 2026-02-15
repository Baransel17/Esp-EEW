#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "*****"; // <-- UPDATE THIS
const char* password = "*****";      // <-- UPDATE THIS

// --- PIN DEFINITIONS ---
#define PIN_LED_RED   25 // Alarm
#define PIN_LED_GREEN 26 // Setup/Calibration 
#define PIN_LED_WHITE 27 // Heartbeat
#define PIN_BUZZER    18 // Alarm

// --- CONFIGURATION ---
#define SERIAL_BAUD_RATE   115200
#define P_WAVE_THRESHOLD   0.30 // Sens
#define ALARM_DURATION     3000 // Alarm stays ON for 3 secs
#define COOLDOWN           1000 // Wait 1 second after alarm to prevent "feedback loop"
#define HEARTBEAT_INTERVAL 5000 // 30 seconds for heartbeat

// --- OBJECTS ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // Create a sensor object with a unique ID(12345)
WebServer server(80);

// Global variable to store the previous reading
float previousMagnitude = 0;
bool firstReading = true; // Flag to handle the first reading

bool isAlarmActive = false;
unsigned long alarmOffTime = 0;
unsigned long lastHeartbeatTime = 0; // Tracks the last heartbeat

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

  // --- JAVASCRIPT: GOOGLE MAPS & AJAX ---
  html += "<script>";
  html += "let map;";
  html += "let stationMarker;";

  // Initialize Google Maps
  html += "function initMap() {";
  html += "  const stationPos = { lat: 0, lng: 0 };"; // Default: ?
  html += "  map = new google.maps.Map(document.getElementById('map'), {";
  html += "    zoom: 12,";
  html += "    center: stationPos,";
  html += "    mapTypeId: 'terrain'";
  html += "  });";
  html += "  stationMarker = new google.maps.Marker({";
  html += "    position: stationPos,";
  html += "    map: map,";
  html += "    title: 'Station 1 (Active)'";
  html += "  });";
  html += "}";
  
  // --- JAVASCRIPT SECTION ---
  // This script runs on your phone/browser. It asks the ESP32 for status every 500ms.
  // Real-time AJAX Check

  html += "setInterval(function() {";
  html += "  fetch('/status').then(response => response.text()).then(data => {";
  html += "    if (data == 'ALARM') {";
  html += "      document.body.style.backgroundColor = '#ffcccc';";
  html += "      document.getElementById('statusText').innerText = 'EARTHQUAKE!';";
  html += "      document.getElementById('statusText').style.color = 'red';";
  // Future idea: We can make the map marker bounce or change color here!
  html += "    } else {";
  html += "      document.body.style.backgroundColor = '#f4f4f4';";
  html += "      document.getElementById('statusText').innerText = 'SAFE';";
  html += "      document.getElementById('statusText').style.color = 'green';";
  html += "    }";
  html += "  });";
  html += "}, 500);"; 
  html += "</script>";

  // Load Google Maps API (Replace YOUR_GOOGLE_MAPS_API_KEY later)
  html += "<script async defer src='https://maps.googleapis.com/maps/api/js?key=YOUR_GOOGLE_MAPS_API_KEY&callback=initMap'></script>";

  html += "</head><body>";
  html += "<div class='container'>";
  html += "<h2>SEISMIC NETWORK</h2>";
  html += "<h3>Node: Station_1</h3>";
  html += "<hr>";
  html += "<h1 id='statusText'>SAFE</h1>"; // This text changes
  html += "<p>System is monitoring P-Waves...</p>";
  html += "<div id='map'></div>"; // Map container
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
  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  // Starting Webserver
  server.on("/", handleRoot); // Load the page
  server.on("/status", handleStatus); // Check status
  server.begin();

  // Starting Calibration
  for(int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH); delay(200);
    digitalWrite(PIN_LED_GREEN, LOW); delay(200);
  }
  Serial.println(">>> SYSTEM ARMED <<<");
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

  // STATE EVALUATION
  if(!isAlarmActive) {
    // We are SAFE. Check for earthquakes. || Calculate DELTA (change)
    float delta = fabs(currentMagnitude - previousMagnitude); //fabs() is float absolute value (turn negatives to positives)

    if(delta > P_WAVE_THRESHOLD) {
      isAlarmActive = true;
      alarmOffTime = millis() + ALARM_DURATION; // Set the deadline for alarm to turn off
      Serial.println(">>> EARTHQUAKE DETECTED! ALARM ON <<<");
    }
  } else {
    // We are in ALARM mode. Check if it's time to stop.
    if ( millis() >= alarmOffTime) {
      isAlarmActive = false; // Turn off the state
      Serial.println("--- ALARM STOPPED. ENTERING COOLDOWN ---");

      // Forve hardware OFF imm
      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_BUZZER, LOW);

      // COOLDOWN: Let the buzzer's physical vibration settle down
      delay(COOLDOWN);

      // Re-calibrate the baseline after the cooldown
      accel.getEvent(&event);
      currentMagnitude = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
    }
  }

  // HARDWARE EXECUTION (Always enforce the current state)
  if(isAlarmActive) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    // --- HEARTBEAT LOGIC (Only when SAFE) ---
    if(millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
      lastHeartbeatTime = millis();
      digitalWrite(PIN_LED_WHITE, HIGH);
      delay(50);
      digitalWrite(PIN_LED_WHITE, LOW);
      Serial.println("... System Heartbeat ...");
    }
  }

  // UPDATE BASELINE (Only when safe, never during alarm)
  if(!isAlarmActive){
    previousMagnitude = currentMagnitude;
  }

  delay(20);
}