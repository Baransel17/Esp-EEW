#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#include <math.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h> //  For ESP32 Hardware UART
#include <WiFiManager.h>  //  For connect to WiFi easly
#include <WiFiClientSecure.h> //  For cloud security (HiveMQ Cloud TLS/SSL)
#include <ArduinoOTA.h> //  For OTA (Over The Air)
#include <esp_task_wdt.h> // For Watchdog Timer

//  --- USER CONFIGURATIONS --- 
const char* mqtt_server = SECRET_MQTT_SERVER;
const char* mqtt_user = SECRET_MQTT_USER;
const char* mqtt_pass = SECRET_MQTT_PASS; 
const char* mqtt_topic = "seismic_network/station_1/status";
const int mqtt_port = 8883;

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
#define DISPLACEMENT_THRESHOLD 4.0  //  Shift in m/s^2 (~0.4g) to consider the device fallen/displaced

//  --- STA/LTA ALGORITHM CONFIGURATION ---
#define STA_WINDOW_SIZE 50  //  Short-Time Average Window (~1 second at 50Hz)
#define LTA_WINDOW_SIZE 500 //  Long-Time Average Window (~10 seconds at 50Hz)
#define STA_LTA_THRESHOLD 2.5 //  Trigger alarm if STA is 4x larger than LTA

//  --- WATCHDOG CONFIGURATION ---
#define WDT_TIMEOUT 5 //  5 seconds timeout for watchdog

//  --- OBJECTS ---
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); //  Create a sensor object with a unique ID(12345)

// WiFiClient espClient;
WiFiClientSecure espClient; //  Secure client for Cloud
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

//  --- BREATHING LED VARIABLES ---
unsigned long lastMqttHeartbeatTime = 0;

// --- CALIBRATION & DISPLACEMENT VARIABLES ---
float calibX = 0.0, calibY = 0.0, calibZ = 0.0;
bool isDeviceDisplaced = false;
int displacementCounter = 0;  // To prevent false triggers during heavy vibration

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
String  createPayload(String statusMessage, float intensity = 0.0) {
  char intensityStr[10];
  dtostrf(intensity, 1, 3, intensityStr);

  String payload = statusMessage + "|" + String(currentLat, 6) + "," + String(currentLng, 6) + "|" + String(intensityStr);
  return payload;
}

//  --- Non-Blocking MQTT Reconnect Function ---
void checkMqttConnection() {
  if(!mqttClient.connected()) {
    unsigned long now = millis();
    //  Try to reconnect every 5 seconds without blocking the sensor loop
    if(now - lastMqttReconnectAttempt > 5000){
      lastMqttReconnectAttempt = now;
      Serial.print("[MQTT] Attempting cloud connection...");

      //  Connect with Username and Password
      if(mqttClient.connect("ESP32_Station_1", mqtt_user, mqtt_pass)) {
        Serial.println(" CONNECTED TO CLOUD!");
        //  Send initial online status with coordinates
        mqttClient.publish(mqtt_topic, createPayload("SYSTEM_ONLINE", 0.0).c_str());
      } else {
        Serial.println(" FAILED, rc=");
        Serial.print(mqttClient.state());
        Serial.println(" try again in 5 seconds.");
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
  ledcSetup(0, 5000, 8);
  ledcAttachPin(PIN_LED_WHITE, 0);
  pinMode(PIN_BUZZER, OUTPUT);
  
  //  Default state: All off
  digitalWrite(PIN_LED_RED, LOW);
  digitalWrite(PIN_LED_GREEN, LOW);
  ledcWrite(0, 0);
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

  //  --- Starting WiFiManager ---
  Serial.println("Starting WiFiManager...");
  digitalWrite(PIN_LED_GREEN, HIGH);  //  Setup mode for WiFi connection

  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(60); //  Timeout for the captive portal

  if(!wifiManager.autoConnect("Seismic_Node_Setup")) {
    Serial.println("Failed to connect and hit timeout. Restarting...");
    delay(3000);
    ESP.restart();  //  If failed to connect restart esp
  }

  Serial.println("\nWiFi Connected!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  //  --- OTA CONFIGURATION ---
  ArduinoOTA.setHostname("Seismic_Station_1");  //  Hostname for the network
  ArduinoOTA.setPassword(SECRET_OTA_PASS); //  Password for OTA uploads

  ArduinoOTA.onStart([]() {
    Serial.println("[OTA] Update Started... Deactivating Watchdog for safety.");
    esp_task_wdt_init(60, true);
    //  Turn off alarm during update
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\n[OTA] Update Finished! Rebooting...");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("[OTA] Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("[OTA] Error[%u]. Restarting system...", error);
    ESP.restart(); // Restart device on OTA error
  });

  ArduinoOTA.begin();
  Serial.println("[OTA] Over-The-Air Update Ready!");

  //  --- CLOUD MQTT SECURITY SETUP ---
  espClient.setInsecure();  //  Bypass SSL certificate validation for development
  mqttClient.setServer(mqtt_server, mqtt_port);

  //  --- GRAVITY CALIBRATION PHASE ---
  Serial.print("--- Starting Gravity Calibration (Do not move device) ---");
  for(int i = 0; i < 50; i++) {
    sensors_event_t event;
    accel.getEvent(&event);
    calibX += event.acceleration.x;
    calibY += event.acceleration.y;
    calibZ += event.acceleration.z;

    //  Blink Green LED during calibration
    digitalWrite(PIN_LED_GREEN, i % 2 == 0 ? HIGH : LOW);
    delay(100);
  }
  calibX /= 50.0;
  calibY /= 50.0;
  calibZ /= 50.0;

  digitalWrite(PIN_LED_GREEN, LOW);
  Serial.print("Calibrated Base Vector -> X: "); Serial.print(calibX);
  Serial.print(" Y: "); Serial.print(calibY);
  Serial.print(" Z: "); Serial.println(calibZ);

  //  Starting Sequence
  for(int i = 0; i < 5; i++) {
    digitalWrite(PIN_LED_GREEN, HIGH); delay(200);
    digitalWrite(PIN_LED_GREEN, LOW); delay(200);
  }

  //  --- HARDWARE WATCHDOG INITIALIZATION ---
  Serial.println("[WDT] Initializing Watchdog Timer...");
  esp_task_wdt_init(WDT_TIMEOUT, true); //  Initialize WDT with timeout and panic mode enabled
  esp_task_wdt_add(NULL); //  Add the current thread (main loop) to the watchdog

  Serial.println(">>> SYSTEM ARMED <<<");
}

void loop(){

  //  --- Handle OTA Update First ---
  ArduinoOTA.handle();

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

  //  --- SMART DISPLACEMENT / FALL DETECTION & AUTO RECOVERY ---
  float deltaX = fabs(event.acceleration.x - calibX);
  float deltaY = fabs(event.acceleration.y - calibY);
  float deltaZ = fabs(event.acceleration.z - calibZ);

  //  1. Condition: The device has moved, is shaking, or has fallen over
  if(deltaX > DISPLACEMENT_THRESHOLD || deltaY > DISPLACEMENT_THRESHOLD || deltaZ > DISPLACEMENT_THRESHOLD){
    if(!isDeviceDisplaced){
      displacementCounter++;
      //  Trigger alarm if it remains inverted for ~2 seconds
      if(displacementCounter > 100){
        isDeviceDisplaced = true;
        displacementCounter = 0;
        Serial.println(">>> FATAL: DEVICE DISPLACED OR FALLEN! <<<");
        if(mqttClient.connected()){
          mqttClient.publish(mqtt_topic, createPayload("DEVICE_DISPLACED", 0.0).c_str());
        }
      }
    }
    else{
      displacementCounter = 0;
    }
  }
  else{
    //  Status 2: The device has returned to its previous operating positions
    if(isDeviceDisplaced){
      displacementCounter++;
      //  If it stays completely still in its original position for ~2 seconds it will recover
      if(displacementCounter > 100){
        isDeviceDisplaced = false;
        displacementCounter = 0;
        Serial.println("--- DEVICE HAS RETURNED TO ITS ORIGINAL LOCATION. ALARM IS BEING SILENCED ---");

        //  We reset the reference point to prevent the seismic algorithm from suddenly jumping out of control
        previousMagnitude = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));

        digitalWrite(PIN_LED_RED, LOW);
        digitalWrite(PIN_BUZZER, LOW);
        
        if(mqttClient.connected()) {
          mqttClient.publish(mqtt_topic, createPayload("SAFE", 0.0).c_str());
        }
      }
    }
    else {
      displacementCounter = 0; // Everything is normal
    }
  }

  //  --- HARDWARE LOCK (Alarm Active Only) ---
  if(isDeviceDisplaced){
    digitalWrite(PIN_LED_RED, HIGH);

    if(millis() % 1000 < 500) {
      digitalWrite(PIN_BUZZER, HIGH);
    } else {
      digitalWrite(PIN_BUZZER, LOW);
    }

    //  --- KICK THE DOG IN LOCKOUT ---
    esp_task_wdt_reset(); //  Keep feeding the watchdog while locked in fallen state

    delay(20);
    return;
  }

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
        mqttClient.publish(mqtt_topic, createPayload("EARTHQUAKE_ALARM", ratio).c_str());
      }
    }
  } else {
    //  ALARM COOLDOWN LOGIC
    if(millis() >= alarmOffTime) {
      isAlarmActive = false;
      Serial.println("--- ALARM STOPPED. ENTERING COOLDOWN ---");

      if(mqttClient.connected()) {
        mqttClient.publish(mqtt_topic, createPayload("SAFE", 0.0).c_str());
      }

      digitalWrite(PIN_LED_RED, LOW);
      digitalWrite(PIN_BUZZER, LOW);
      delay(COOLDOWN);

      //  Re-calibrate baseline
      accel.getEvent(&event);
      currentMagnitude = sqrt(pow(event.acceleration.x, 2) + pow(event.acceleration.y, 2) + pow(event.acceleration.z, 2));
      previousMagnitude = currentMagnitude;

      //  --- PRIME BUFFERS WITH AMBIENT NOISE ---
      //  Instead of resetting the buffers, we fill them with a low noise level. 
      //  This way, the system doesn't go blank for 10 seconds; it starts listening immediately.
      float ambientNoise = 0.015;

      staSum = ambientNoise * STA_WINDOW_SIZE;
      ltaSum = ambientNoise * LTA_WINDOW_SIZE;

      for(int i = 0; i < STA_WINDOW_SIZE; i++) staBuffer[i] = ambientNoise;
      for(int i = 0; i < LTA_WINDOW_SIZE; i++) ltaBuffer[i] = ambientNoise;

      staIndex = 0;
      ltaIndex = 0;
      
      //  We make the system appear fully loaded and ready instantly
      sampleCount = LTA_WINDOW_SIZE;
      firstReading = true;
      Serial.println("--- MEMORY PRIMED. LISTENING FOR NEW EVENTS INSTANTLY ---");
    }
  }

  //  HARDWARE EXECUTION (Always enforce the current state)
  if(isAlarmActive) {
    digitalWrite(PIN_LED_RED, HIGH);
    digitalWrite(PIN_BUZZER, HIGH);
  } else {
    digitalWrite(PIN_LED_RED, LOW);
    digitalWrite(PIN_BUZZER, LOW);

    //  --- APPLE BREATHING LED & MQTT HEARTBEAT (Only when SAFE) ---
    //  Create a breathing cycle of 4000ms
    float phase = (millis() % 4000) / 4000.0 * 2.0 * PI;

    //  Apple's e^sin(x) based natural breathing formula:
    int pwmValue = (exp(sin(phase)) - 0.36787944) * 108.0;

    //  Prevent it from going outside the boundaries and write to the LED
    pwmValue = constrain(pwmValue, 0, 255);
    ledcWrite(0, pwmValue);

    //  Network HEARTBEAT (MQTT)
    //  While the LED is constantly breathing, we only ping the network every 5 seconds.
    if(millis() - lastMqttHeartbeatTime >= HEARTBEAT_INTERVAL){
      lastMqttHeartbeatTime = millis();

      //  Print GPS status to Serial Monitor
      Serial.print("... System Heartbeat ... [Lat: ");
      Serial.print(currentLat, 6);
      Serial.print(", Lng: ");
      Serial.print(currentLng, 6);
      Serial.println("]");

      //  Publish Heartbeat with GPS coordinates
      if(mqttClient.connected()){
        mqttClient.publish(mqtt_topic, createPayload("HEARTBEAT", 0.0).c_str());
      }
    }
    
  }

  //  UPDATE BASELINE (Only when safe, never during alarm)
  if(!isAlarmActive){
    previousMagnitude = currentMagnitude;
  }

  //  --- RESET THE DOG ---
  esp_task_wdt_reset();

  delay(20);
}