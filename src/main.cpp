#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <WiFi.h>
#include <WebServer.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h> //  For ESP32 Hardware UART

//  --- USER CONFIGURATIONS --- 
const char* ssid = "*****";                                   // <-- UPDATE THIS
const char* password = "*****";                               // <-- UPDATE THIS
const char* mqtt_server = "*****";                            // <-- Local IP Address
const char* mqtt_topic = "seismic_network/station_1/status";  // MQTT Topic

//  --- PIN DEFINITIONS ---
#define PIN_LED_RED   25 // Alarm
#define PIN_LED_GREEN 26 // Setup/Calibration 
#define PIN_LED_WHITE 27 // Heartbeat
#define PIN_BUZZER    18 // Alarm

//  --- GPS PIN DEFINITIONS
#define GPS_RX_PIN 16
#define GPS_TX_PIN 17
#define GPS_BAUD_RATE 9600

//  --- CONFIGURATION ---
#define SERIAL_BAUD_RATE   115200
#define ALARM_DURATION     3000 //  Alarm stays ON for 3 secs
#define COOLDOWN           1000 //  Wait 1 second after alarm to prevent "feedback loop"
#define HEARTBEAT_INTERVAL 5000 //  5 seconds for heartbeat

//  --- STA/LTA ALGORITHM CONFIGURATION ---
#define STA_WINDOW_SIZE 50  //  Short-Time Average Window (~1 second at 50Hz)
#define LTA_WINDOW_SIZE 500 //  Long-Time Average Window (~10 seconds at 50Hz)
#define STA_LTA_THRESHOLD 2.5 //  Trigger alarm if STA is 4x larger than LTA

// --- OBJECTS ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); //  Create a sensor object with a unique ID(12345)
WebServer server(80);
WiFiClient espClient;               //  WiFi Client for MQTT
PubSubClient mqttClient(espClient); //  MQTT Client
TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  //  Using UART2 for GPS communication

//  --- GLOBAL VARIABLES ---
float previousMagnitude = 0;
bool firstReading = true; //  Flag to handle the first reading

bool isAlarmActive = false;
unsigned long alarmOffTime = 0;
unsigned long lastHeartbeatTime = 0;        //  Tracks the last heartbeat
unsigned long lastMqttReconnectAttempt = 0; //  For non-blocking reconnect

//  Variables to store the latest valid GPS coordinates
double currentLat = 0.0;
double currentLng = 0.0;

//  --- STA/LTA VARIABLES ---
float staBuffer[STA_WINDOW_SIZE];
float ltaBuffer[LTA_WINDOW_SIZE];
float staSum = 0;
float ltaSum = 0;
int staIndex = 0;
int ltaIndex = 0;
int sampleCount = 0;  //  Tracks initial buffer fill

//  --- HELPER FUNCTION: CREATE MQTT PAYLOAD ---
//  Combines status and GPS data -> Format: "STATUS|LAT,LNG"
String  createPayload(String statusMessage) {
  String payload = statusMessage + "|" + String(currentLat, 6) + "," + String(currentLng, 6);
  return payload;
}

//  Web page
String getHTML() {
  //  R"rawliteral(...)rawliteral" is for optimized ESP32 memory
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name='viewport' content='width=device-width, initial-scale=1'>
  <style>
    body { font-family: sans-serif; text-align: center; margin-top: 20px; transition: background-color 0.5s; background-color: #f4f4f4;}
    .container { background-color: white; padding: 20px; border-radius: 8px; box-shadow: 0 4px 8px rgba(0,0,0,0.1); width: 90%; max-width: 600px; margin: auto; }
    h1 { font-size: 50px; margin: 10px 0; }
    #statusText { font-weight: bold; }
    #map { height: 350px; width: 100%; margin-top: 20px; border-radius: 5px; background-color: #e0e0e0; border: 1px solid #ccc; } 
  </style>

  <script>
    let map;
    let stationMarker;
    let isMapInitialized = false;
    
    function initMap() {
      const defaultPos = { lat: 0.0, lng: 0.0 };
      map = new google.maps.Map(document.getElementById('map'), {
        zoom: 2,
        center: defaultPos,
        mapTypeId: 'terrain'
      });
      stationMarker = new google.maps.Marker({
        position: defaultPos,
        map: map,
        title: 'Station 1 (Active)'
      });
    }

    setInterval(function() {
      fetch('/status').then(response => response.text()).then(data => {
        if (data == 'ALARM') {
          // Earthquake state colors
          document.body.style.backgroundColor = '#ffcccc';
          document.getElementById('statusText').innerText = 'EARTHQUAKE!';
          document.getElementById('statusText').style.color = 'red';
          
          // Animation: Start bouncing if not bouncing
          if (stationMarker && stationMarker.getAnimation() == null) {
            stationMarker.setAnimation(google.maps.Animation.BOUNCE);
          }
        } else {
          // Safe state colors
          document.body.style.backgroundColor = '#f4f4f4';
          document.getElementById('statusText').innerText = 'SAFE';
          document.getElementById('statusText').style.color = 'green';
          
          // Animation: Stop bouncing
          if (stationMarker && stationMarker.getAnimation() != null) {
            stationMarker.setAnimation(null);
          }
        }
      });
      // Real-Time GPS location
      fetch('/gps').then(response => response.json()).then(data => {
        if(data.lat !== 0.0 && data.lng !== 0.0) {
          const newPos = { lat: data.lat, lng: data.lng };
          stationMarker.setPosition(newPos);

          if(!isMapInitialized) {
            map.setCenter(newPos);
            map.setZoom(14);
            isMapInitialized = true;
          }
        }
      }).catch(err => console.log('Waiting for GPS sync...'));

    }, 1000);
  </script>
  <!-- Load Google Maps API (Replace YOUR_GOOGLE_MAPS_API_KEY later) -->      
  <script async defer src="https://maps.googleapis.com/maps/api/js?key=YOUR_GOOGLE_MAPS_API_KEY&callback=initMap"></script>
</head>
<body>
  <div class='container'>
    <h2>SEISMIC NETWORK</h2>
    <h3>Node: Station_1</h3>
    <hr>
    <h1 id='statusText'>SAFE</h1>
    <p>System is monitoring P-Waves...</p>
    <div id='map'>
      <p style="padding-top: 150px; color: #666;">Loading Map...</p>
    </div>
  </div>
</body>
</html>
  )rawliteral";
  
  return html;
}

void handleStatus(){
  if(isAlarmActive) {
    server.send(200, "text/plain", "ALARM");
  } else {
    server.send(200, "text/plain", "SAFE");
  }
}

void handleGPS(){
  String json = "{\"lat\": " + String(currentLat, 6) + ", \"lng\": " + String(currentLng, 6) + "}";
  server.send(200, "application/json", json);
}

//  Webserver Functions
void handleRoot(){
  server.send(200, "text/html", getHTML());
}

//  --- Non-Blocking MQTT Reconnect Function ---
void checkMqttConnection() {
  if(!mqttClient.connected()) {
    unsigned long now = millis();
    //  Try to reconnect every 5 seconds without blocking the sensor loop
    if(now - lastMqttReconnectAttempt > 5000){
      lastMqttReconnectAttempt = now;
      Serial.print("[MQTT] Attempting connection...");
      if(mqttClient.connect("ESP32_Station_1")) {
        Serial.println(" CONNECTED!");
        //  Send initial online status with coordinates
        mqttClient.publish(mqtt_topic, createPayload("SYSTEM_ONLINE").c_str());
      } else {
        Serial.println(" FAILED.");
      }
    }
  } else {
    mqttClient.loop();  //  Keep the connection alive
  }
}

void setup(){
  Serial.begin(SERIAL_BAUD_RATE); // Initialize Serial comm

  //  Initialize GPS Serial Communication
  gpsSerial.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  Serial.println("GPS Serial Initialized");

  //  Pin configurations
  pinMode(PIN_LED_RED, OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_WHITE, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);
  
  //  Default state: All off
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  digitalWrite(PIN_LED_WHITE, LOW);
  digitalWrite(PIN_BUZZER, LOW);

  //  Initialize the sensor
  if(!accel.begin()){
    Serial.println("ERROR: No ADXL345 detected! -- Check your wiring!");
    while (1);
  }
  accel.setRange(ADXL345_RANGE_2_G); // Set sensor range

  //  Initialize STA/LTA buffers with 0
  for(int i = 0; i < STA_WINDOW_SIZE; i++) staBuffer[i] = 0;
  for(int i = 0; i < LTA_WINDOW_SIZE; i++) ltaBuffer[i] = 0;

  //  Wifi Start
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

  //  --- Setup MQTT Server
  mqttClient.setServer(mqtt_server, 1883);

  //  Webserver Routes
  server.on("/", handleRoot); // Load the page
  server.on("/status", handleStatus); // Check status
  server.on("/gps", handleGPS); //  Route for fetching GPS coordinates
  server.begin();

  //  Starting Calibration
  for(int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH); delay(200);
    digitalWrite(PIN_LED_GREEN, LOW); delay(200);
  }
  Serial.println(">>> SYSTEM ARMED <<<");
}

void loop(){

  server.handleClient();  //  Handle Requests
  checkMqttConnection();  //  Keep MQTT alive without blocking

  //  Read incoming GPS data without blocking the loop
  while(gpsSerial.available() > 0) {
    if(gps.encode(gpsSerial.read())){
      //  If a valid location is found, update the global variables
      if(gps.location.isValid()) {
        currentLat = gps.location.lat();
        currentLng = gps.location.lng();
      }
    }
  }

  //  Get a new sensor event
  sensors_event_t event;
  accel.getEvent(&event);

  //  Extract Axis Data
  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;
  float currentMagnitude = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)); //  APPLY PYTHAGORAS THEOREM

  if (firstReading) {
    previousMagnitude = currentMagnitude;
    firstReading = false;
    return; //  Skip the rest of the loop once
  }

  //  --- STA/LTA CALCULATION ---
  if(!isAlarmActive) {
    float signal = fabs(currentMagnitude - previousMagnitude);

    //  Update Short-Time Average (STA)
    staSum -= staBuffer[staIndex];
    staBuffer[staIndex] = signal;
    staSum += signal;
    staIndex = (staIndex + 1) % STA_WINDOW_SIZE;
    float currentSTA = staSum / STA_WINDOW_SIZE;

    //  Update Long-Time Average (LTA)
    ltaSum -= ltaBuffer[ltaIndex];
    ltaBuffer[ltaIndex] = signal;
    ltaSum += signal;
    ltaIndex = (ltaIndex + 1) % LTA_WINDOW_SIZE;

    //  Prevent division by zero and handle startup phase
    int ltaDivisor = (sampleCount < LTA_WINDOW_SIZE) ? sampleCount : LTA_WINDOW_SIZE;
    float currentLTA = (ltaDivisor > 0) ? (ltaSum / ltaDivisor) : 0.001;

    if(sampleCount < LTA_WINDOW_SIZE) sampleCount++;

    //  Calculate Ratio
    float ratio = 0;
    if(currentLTA > 0.0001) {
      ratio = currentSTA / currentLTA;
    }

    //  STATE EVALUATION (Only evaluate after buffer is reasonably filled)
    if(sampleCount >= LTA_WINDOW_SIZE && ratio > STA_LTA_THRESHOLD) {
      isAlarmActive = true;
      alarmOffTime = millis() + ALARM_DURATION;
      Serial.print(">>> EARTHQUAKE DETECTED! Ratio: ");
      Serial.print(ratio);
      Serial.println(" ALARM ON <<<");

      if(mqttClient.connected()) {
        mqttClient.publish(mqtt_topic, createPayload("EARTHQUAKE_ALARM").c_str());
      }
    }
  } else {
    //  ALARM COOLDOWN LOGIC
    if(millis() >= alarmOffTime) {
      isAlarmActive = false;
      Serial.println("--- ALARM STOPPED. ENTERING COOLDOWN ---");

      if(mqttClient.connected()) {
        mqttClient.publish(mqtt_topic, createPayload("SAFE").c_str());
      }

      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      delay(COOLDOWN);

      //  Re-calibrate baseline
      accel.getEvent(&event);
      currentMagnitude = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
      previousMagnitude = currentMagnitude;

      //  Flush Buffer for Buffer Contamination
      staSum = 0;
      ltaSum = 0;
      staIndex = 0;
      ltaIndex = 0;
      for(int i = 0; i < STA_WINDOW_SIZE; i++) staBuffer[i] = 0;
      for(int i = 0; i < LTA_WINDOW_SIZE; i++) ltaBuffer[i] = 0;

      sampleCount = 0;
      Serial.println("--- MEMORY CLEARED. LISTENING FOR NEW EVENTS ---");
    }
  }

  //  HARDWARE EXECUTION (Always enforce the current state)
  if(isAlarmActive) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    //  --- HEARTBEAT LOGIC (Only when SAFE) ---
    if(millis() - lastHeartbeatTime >= HEARTBEAT_INTERVAL) {
      lastHeartbeatTime = millis();
      digitalWrite(PIN_LED_WHITE, HIGH);
      delay(50);
      digitalWrite(PIN_LED_WHITE, LOW);
      
      //  Print GPS status to Serial Monitor for debugging
      Serial.print("... System Heartbeat ... [Lat: ");
      Serial.print(currentLat, 6);
      Serial.print(", Lng: ");
      Serial.print(currentLng, 6);
      Serial.println("]");

      //  Publish Heartbeat with GPS coordinates
      if(mqttClient.connected()){
        mqttClient.publish(mqtt_topic, createPayload("HEARTBEAT").c_str());
      }
    }
  }

  //  UPDATE BASELINE (Only when safe, never during alarm)
  if(!isAlarmActive){
    previousMagnitude = currentMagnitude;
  }

  delay(20);
}