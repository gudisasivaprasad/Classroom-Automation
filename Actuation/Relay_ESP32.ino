#include <WiFi.h>
#include <WiFiUdp.h>
#include "DHT.h"
#include <Arduino.h>
#include <Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

const char* apiKey = "AIzaSyBB-a2eihPKRibslI80FTOdldwwx30FvGc";
const char* databaseURL = "https://smart-classroom-c68f0-default-rtdb.asia-southeast1.firebasedatabase.app";

// -------------------- WiFi Credentials --------------------
const char* ssid = "Siva";
const char* password = "123456789";

// -------------------- UDP Configuration -------------------
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
bool signupOk=false;
WiFiUDP udp;
const int masterUdpPort = 4210;       // Master UDP port
const char* slaveIp = "192.168.96.63"; // IP of the Slave ESP01
const int slaveUdpPort = 4220;        // Slave UDP port
const char* pythonServerIp = "192.168.96.10"; // IP of the Python Server
const int pythonServerPort = 4230;    // Python Server UDP port
// Added new ESP32 configuration
const char* anotherEsp32Ip = "192.168.96.167"; // IP of the other ESP32
const int anotherEsp32Port = 4520;    // Port of the other ESP32
char incomingPacket[255];             // Buffer for incoming packets

// -------------------- Threshold -------------------
const int luxThreshold = 300;

// -------------------- Relay Pins for Lighting System -------------------
const int relay1Pin = 25;
const int relay2Pin = 26;
const int relay3Pin = 27;
const int relay4Pin = 33;

// -------------------- DHT Sensor configuration -------------------
#define DHTPIN 4
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// -------------------- Switch pins for Fan Control -------------------
#define SWITCH_1 5
#define SWITCH_2 16
#define SWITCH_3 17
#define SWITCH_4 18
#define SWITCH_5 19  // Switch for enabling/disabling DHT11 automation

// -------------------- Relay pins for Fan Control -------------------
#define FAN_RELAY_1 21
#define FAN_RELAY_2 22
#define FAN_RELAY_3 23

// -------------------- Temperature thresholds for automatic fan control -------------------
#define TEMP_LOW 23.0    // Below this temperature, relay 1 on
#define TEMP_MEDIUM 24.0 // Between 24 and 27, relay 2 on
#define TEMP_HIGH 27.0   // Between 27 and 32, relay 1 and 2 on
#define TEMP_VERY_HIGH 32.0 // Above 32, relay 3 on

// -------------------- Lux Sensor Configuration -------------------
struct LuxSensor {
  const char* ip;
  int port;
  int lastLuxValue;
  unsigned long lastCheckTime;
};

LuxSensor luxSensors[4] = {
  {"192.168.96.207", 4240, 0, 0},  // Sensor for Light 1
  {"192.168.96.36", 4241, 0, 0},  // Sensor for Light 2
  {"192.168.96.11", 4242, 0, 0},  // Sensor for Light 3
  {"192.168.96.171", 4243, 0, 0}  // Sensor for Light 4
};

// -------------------- State Flags for Lighting System -------------------
bool personDetected = false;          // Flag to track if any person is detected
bool personDetectedCam1 = false;      // Person detected by camera 1 (lights 1-2)
bool personDetectedCam2 = false;      // Person detected by camera 2 (lights 3-4)
bool activeLights[4] = {false, false, false, false};  // Track which lights are currently active
unsigned long lastStatusRequestTime = 0;  // Track when the last status request was sent
const unsigned long STATUS_REQUEST_INTERVAL = 2000;  // Request status every 2 seconds


// -------------------- Variables for Fan Control System -------------------
bool switchState1 = false;
bool switchState2 = false;
bool switchState3 = false;
bool switchState4 = false;
bool switchState5 = false; // For DHT11 automation enable/disable
bool manualControl = false;

// -------------------- Timer for Fan Control System -------------------
unsigned long lastFanControlTime = 0;
const unsigned long FAN_CONTROL_INTERVAL = 2000; // Check fan control every 2 seconds
void fb_upload();

void setup() {
  Serial.begin(115200);

  // -------------------- Connect to Wi-Fi --------------------
  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  
  // Wait until connected
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected!");
  Serial.print("ESP32 IP Address: ");
  Serial.println(WiFi.localIP());

  // -------------------- Start UDP ---------------------------
  udp.begin(masterUdpPort);
  Serial.printf("Master listening on UDP port %d\n", masterUdpPort);

  // Initialize lighting system relays as outputs
  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);
  pinMode(relay3Pin, OUTPUT);
  pinMode(relay4Pin, OUTPUT);

  // Turn off all lighting relays initially
  turnOffAllLightRelays();

  // Initialize DHT sensor
  dht.begin();
  Serial.println(F("DHT11 sensor initialized"));

  // Configure switch pins as inputs with pullup resistors
  pinMode(SWITCH_1, INPUT_PULLUP);
  pinMode(SWITCH_2, INPUT_PULLUP);
  pinMode(SWITCH_3, INPUT_PULLUP);
  pinMode(SWITCH_4, INPUT_PULLUP);
  pinMode(SWITCH_5, INPUT_PULLUP);
  
  // Configure fan relay pins as outputs
  pinMode(FAN_RELAY_1, OUTPUT);
  pinMode(FAN_RELAY_2, OUTPUT);
  pinMode(FAN_RELAY_3, OUTPUT);
  
  // Initialize fan relays to OFF state
  digitalWrite(FAN_RELAY_1, LOW);
  digitalWrite(FAN_RELAY_2, LOW);
  digitalWrite(FAN_RELAY_3, LOW);
  
  Serial.println("ESP32 Smart Lighting and Fan Control System initialized");
  config.api_key=apiKey;
  config.database_url=databaseURL;
if (Firebase.signUp(&config, &auth, "", "")) {
        Serial.println("Firebase sign-up successful.");
        signupOk = true;
    } else {
        Serial.println("Firebase sign-up failed.");
        Serial.printf("Error: %s\n", config.signer.signupError.message.c_str());
    }

    Firebase.begin(&config, &auth);
    Firebase.reconnectWiFi(true);
}

void loop() {
  // Handle lighting control system
  handleLightingSystem();
  
  // Handle fan control system
  unsigned long currentTime = millis();
  if (currentTime - lastFanControlTime >= FAN_CONTROL_INTERVAL) {
    handleFanControlSystem();
    lastFanControlTime = currentTime;
    
    // Display system status every fan control interval
    displaySystemStatus();
  }
  
  delay(100); // Short delay for loop stability
}

// -------------------- Lighting System Functions --------------------

void handleLightingSystem() {
  // Check for person detection first
  if (!personDetected) {
    // Request data from ultrasonic sensor
    sendToSlave("REQUEST_ULTRASONIC");
    delay(100); // Short wait for response
    
    // Listen for response from ultrasonic sensor
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
      if (len > 0) {
        incomingPacket[len] = '\0'; // Null-terminate the received data
      }
      
      String response = String(incomingPacket);
      Serial.printf("Message from Slave: %s\n", response.c_str());
      
      // Check if person is detected
      if (response.startsWith("PERSON_DETECTED")) {
        personDetected = true;
        
        // Check which camera detected the person
        if (response == "PERSON_DETECTED_CAM1") {
          personDetectedCam1 = true;
          Serial.println("Person detected by Camera 1.");
        } 
        else if (response == "PERSON_DETECTED_CAM2") {
          personDetectedCam2 = true;
          Serial.println("Person detected by Camera 2.");
        }
        else {
          // Default case for backward compatibility
          personDetectedCam1 = true;
          personDetectedCam2 = true;
          Serial.println("Person detected. Camera not specified.");
        }
        
        // Forward the person detection status to another ESP32
        sendToAnotherEsp32(response.c_str());
      }
    }
  }
  
  // If person is detected, periodically request light status
  if (personDetected) {
    unsigned long currentTime = millis();
    
    // Check if it's time to request light status
    if (currentTime - lastStatusRequestTime >= STATUS_REQUEST_INTERVAL) {
      sendToPython("REQUEST_LIGHT_STATUS");
      lastStatusRequestTime = currentTime;
      Serial.println("Requesting light status from Python...");
    }
    
    // Check for light status updates
    int packetSize = udp.parsePacket();
    if (packetSize > 0) {
      int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
      if (len > 0) {
        incomingPacket[len] = '\0'; // Null-terminate the received data
      }
      
      String response = String(incomingPacket);
      Serial.printf("Response received: %s\n", response.c_str());
      
      // Handle light commands
      if (response.startsWith("light") && response.endsWith("_ON")) {
        int lightNum = response.substring(5, 6).toInt();
        if (lightNum >= 1 && lightNum <= 4) {
          Serial.printf("Turning ON light %d\n", lightNum);
          checkLuxAndControlRelay(lightNum);
        }
      } 
      else if (response.startsWith("light") && response.endsWith("_OFF")) {
        int lightNum = response.substring(5, 6).toInt();
        if (lightNum >= 1 && lightNum <= 4) {
          Serial.printf("Turning OFF light %d\n", lightNum);
          turnOffLightRelay(lightNum);
          activeLights[lightNum - 1] = false;
        }
      }
      else if (response.startsWith("STATUS:")) {
        String statusPart = response.substring(7); // Remove "STATUS:" prefix
        
        // If no lights are active, check if person is still detected
        if (statusPart == "NO_LIGHTS") {
          // Forward the NO_LIGHTS message to another ESP32
          Serial.println("NO_LIGHTS status received from Python. Forwarding to another ESP32.");
          sendToAnotherEsp32("NO_LIGHTS");
          
          // Check if we still have a person detection
          sendToSlave("CHECK_PERSON");
          delay(100);
          
          int slaveResponse = udp.parsePacket();
          if (slaveResponse > 0) {
            int slaveLen = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
            if (slaveLen > 0) {
              incomingPacket[slaveLen] = '\0';
            }
            
            String personStatus = String(incomingPacket);
            if (personStatus == "NO_PERSON") {
              Serial.println("No persons detected. Resetting light system.");
              personDetected = false;
              personDetectedCam1 = false;
              personDetectedCam2 = false;
              turnOffAllLightRelays();
              for (int i = 0; i < 4; i++) {
                activeLights[i] = false;
              }
              
              // Forward the no person detection status to another ESP32
              sendToAnotherEsp32("NO_PERSON");
            }
          }
        }
        else {
          // Process status information for multiple lights
          // The format is "STATUS:light1_ON,light2_OFF,light3_ON,light4_OFF"
          int lightIndex = 1;
          int startPos = 7; // Start after "STATUS:"
          
          while (lightIndex <= 4 && startPos < response.length()) {
            int commaPos = response.indexOf(',', startPos);
            if (commaPos == -1) commaPos = response.length();
            
            String lightStatus = response.substring(startPos, commaPos);
            if (lightStatus.endsWith("_ON")) {
              int lightNum = lightStatus.substring(5, 6).toInt();
              if (lightNum >= 1 && lightNum <= 4) {
                Serial.printf("Status update: Light %d should be ON\n", lightNum);
                checkLuxAndControlRelay(lightNum);
              }
            } 
            else if (lightStatus.endsWith("_OFF")) {
              int lightNum = lightStatus.substring(5, 6).toInt();
              if (lightNum >= 1 && lightNum <= 4) {
                Serial.printf("Status update: Light %d should be OFF\n", lightNum);
                turnOffLightRelay(lightNum);
                activeLights[lightNum - 1] = false;
              }
            }
            
            startPos = commaPos + 1;
            lightIndex++;
          }
        }
      }
      else if (response == "NO_PERSON") {
        Serial.println("No persons detected by Python. Checking with ultrasonic sensor.");
        sendToSlave("CHECK_PERSON");
        delay(100);
        
        int slaveResponse = udp.parsePacket();
        if (slaveResponse > 0) {
          int slaveLen = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
          if (slaveLen > 0) {
            incomingPacket[slaveLen] = '\0';
          }
          
          String personStatus = String(incomingPacket);
          if (personStatus == "NO_PERSON") {
            Serial.println("No persons detected by ultrasonic sensor either. Resetting light system.");
            personDetected = false;
            personDetectedCam1 = false;
            personDetectedCam2 = false;
            turnOffAllLightRelays();
            for (int i = 0; i < 4; i++) {
              activeLights[i] = false;
            }
            
            // Forward the no person detection status to another ESP32
            sendToAnotherEsp32("NO_PERSON");
          }
        }
      }
      // Handle camera-specific detection
      else if (response == "CAM1_PERSON") {
        personDetectedCam1 = true;
        personDetected = true;
        Serial.println("Person detected by Camera 1.");
      }
      else if (response == "CAM2_PERSON") {
        personDetectedCam2 = true;
        personDetected = true;
        Serial.println("Person detected by Camera 2.");
      }
      else if (response == "CAM1_NO_PERSON") {
        personDetectedCam1 = false;
        // Only set overall personDetected to false if both cameras report no person
        if (!personDetectedCam2) {
          personDetected = false;
        }
        Serial.println("No person detected by Camera 1.");
      }
      else if (response == "CAM2_NO_PERSON") {
        personDetectedCam2 = false;
        // Only set overall personDetected to false if both cameras report no person
        if (!personDetectedCam1) {
          personDetected = false;
        }
        Serial.println("No person detected by Camera 2.");
      }
    }
  }
}

// Send messages to another ESP32
void sendToAnotherEsp32(const char* message) {
  udp.beginPacket(anotherEsp32Ip, anotherEsp32Port);
  udp.write((uint8_t*)message, strlen(message));
  udp.endPacket();
  Serial.printf("Message sent to Another ESP32: %s\n", message);
}

void sendToSlave(const char* message) {
  udp.beginPacket(slaveIp, slaveUdpPort);
  udp.write((uint8_t*)message, strlen(message));
  udp.endPacket();
  Serial.printf("Message sent to Slave: %s\n", message);
}

void sendToPython(const char* message) {
  udp.beginPacket(pythonServerIp, pythonServerPort);
  udp.write((uint8_t*)message, strlen(message));
  udp.endPacket();
  Serial.printf("Message sent to Python: %s\n", message);
}

void turnOnLightRelay(int relayNum) {
  switch (relayNum) {
    case 1: 
      digitalWrite(relay1Pin, LOW); // LOW activates the relay (active low)
      Serial.printf("RELAY ACTION: Setting relay1Pin (pin %d) to LOW (ON)\n", relay1Pin);
      break;
    case 2: 
      digitalWrite(relay2Pin, LOW);
      Serial.printf("RELAY ACTION: Setting relay2Pin (pin %d) to LOW (ON)\n", relay2Pin);
      break;
    case 3: 
      digitalWrite(relay3Pin, LOW);
      Serial.printf("RELAY ACTION: Setting relay3Pin (pin %d) to LOW (ON)\n", relay3Pin);
      break;
    case 4: 
      digitalWrite(relay4Pin, LOW);
      Serial.printf("RELAY ACTION: Setting relay4Pin (pin %d) to LOW (ON)\n", relay4Pin);
      break;
    default: 
      Serial.println("Invalid Light Relay Number"); 
      return; // Exit without updating activeLights
  }
  
  if (relayNum >= 1 && relayNum <= 4) {
    activeLights[relayNum - 1] = true;
    Serial.printf("Light Relay %d turned ON and marked as active\n", relayNum);
  }
}

void turnOffLightRelay(int relayNum) {
  switch (relayNum) {
    case 1: 
      digitalWrite(relay1Pin, HIGH); // HIGH deactivates the relay (active low)
      Serial.printf("RELAY ACTION: Setting relay1Pin (pin %d) to HIGH (OFF)\n", relay1Pin);
      break;
    case 2: 
      digitalWrite(relay2Pin, HIGH);
      Serial.printf("RELAY ACTION: Setting relay2Pin (pin %d) to HIGH (OFF)\n", relay2Pin);
      break;
    case 3: 
      digitalWrite(relay3Pin, HIGH);
      Serial.printf("RELAY ACTION: Setting relay3Pin (pin %d) to HIGH (OFF)\n", relay3Pin);
      break;
    case 4: 
      digitalWrite(relay4Pin, HIGH);
      Serial.printf("RELAY ACTION: Setting relay4Pin (pin %d) to HIGH (OFF)\n", relay4Pin);
      break;
    default: 
      Serial.println("Invalid Light Relay Number"); 
      return; // Exit without updating activeLights
  }
  
  if (relayNum >= 1 && relayNum <= 4) {
    activeLights[relayNum - 1] = false;
    Serial.printf("Light Relay %d turned OFF and marked as inactive\n", relayNum);
  }
}

void turnOffAllLightRelays() {
  digitalWrite(relay1Pin, HIGH);
  digitalWrite(relay2Pin, HIGH);
  digitalWrite(relay3Pin, HIGH);
  digitalWrite(relay4Pin, HIGH);
  Serial.println("All light relays are turned OFF (pins set to HIGH)");
}

void sendLuxRequest(const char* sensorIp, int sensorPort) {
  udp.beginPacket(sensorIp, sensorPort);
  udp.write((uint8_t*)"REQUEST_LUX", strlen("REQUEST_LUX"));
  udp.endPacket();
  Serial.printf("Lux request sent to sensor at %s:%d\n", sensorIp, sensorPort);
}

void checkLuxAndControlRelay(int lightNum) {
  if (lightNum < 1 || lightNum > 4) {
    Serial.println("Invalid light number");
    return;
  }
  
  int sensorIndex = lightNum - 1;
  LuxSensor* sensor = &luxSensors[sensorIndex];
  
  // Check if it's time to refresh the lux value (every 5 seconds)
  unsigned long currentTime = millis();
  if (currentTime - sensor->lastCheckTime > 5000 || sensor->lastCheckTime == 0) {
    Serial.printf("Time to refresh lux value for Light %d (sensor at %s:%d)\n", 
                  lightNum, sensor->ip, sensor->port);
    
    bool luxResponseReceived = false;
    int retryCount = 0;
    const int maxRetries = 3;
    
    while (!luxResponseReceived && retryCount < maxRetries) {
      sendLuxRequest(sensor->ip, sensor->port);
      
      // Wait for response with timeout
      unsigned long startTime = millis();
      bool responseTimeout = false;
      
      while (!responseTimeout && !luxResponseReceived) {
        delay(50);
        int packetSize = udp.parsePacket();
        
        if (packetSize > 0) {
          int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
          if (len > 0) {
            incomingPacket[len] = '\0';
          }
          
          String luxResponse = String(incomingPacket);
          Serial.printf("Lux value from Sensor%d: %s\n", lightNum, luxResponse.c_str());
          
          int luxValue = luxResponse.toInt();
          if (luxValue > 0 || luxResponse == "0") { // Valid response (including zero lux)
            luxResponseReceived = true;
            sensor->lastLuxValue = luxValue;
            sensor->lastCheckTime = currentTime;
            Serial.printf("Valid lux value of %d received for Light %d\n", luxValue, lightNum);
          } else {
            Serial.printf("Invalid lux value received: '%s'\n", luxResponse.c_str());
          }
        }
        
        // Check for timeout after 1 second
        if (millis() - startTime > 1000) {
          responseTimeout = true;
          Serial.printf("Timeout waiting for lux sensor %d response\n", lightNum);
        }
      }
      
      retryCount++;
      if (!luxResponseReceived && retryCount < maxRetries) {
        Serial.printf("Retrying lux request for Sensor%d (attempt %d of %d)\n", lightNum, retryCount + 1, maxRetries);
      }
    }
    
    if (!luxResponseReceived) {
      Serial.printf("Failed to get lux value from Sensor%d after %d attempts\n", lightNum, maxRetries);
      // Use previous value if available
      if (sensor->lastCheckTime > 0) {
        Serial.printf("Using previous lux value: %d\n", sensor->lastLuxValue);
      } else {
        // If no previous value, assume we need the light
        sensor->lastLuxValue = 0;
        Serial.printf("No previous lux value available for Light %d. Setting to 0 to ensure light turns on\n", lightNum);
      }
    }
  } else {
    Serial.printf("Using cached lux value of %d for Light %d (last updated %lums ago)\n", 
                 sensor->lastLuxValue, lightNum, (currentTime - sensor->lastCheckTime));
  }
  
  // Control the relay based on lux value
  Serial.printf("DECISION: Light %d - Current lux: %d, Threshold: %d\n", 
                lightNum, sensor->lastLuxValue, luxThreshold);
  
  if (sensor->lastLuxValue < luxThreshold) {
    // Light needed - turn it on
    Serial.printf("Light %d: Lux value %d is below threshold %d - TURNING ON\n", 
                  lightNum, sensor->lastLuxValue, luxThreshold);
    turnOnLightRelay(lightNum);
  } else {
    // Enough ambient light - keep light off
    Serial.printf("Light %d: Lux value %d is above threshold %d - KEEPING OFF\n", 
                  lightNum, sensor->lastLuxValue, luxThreshold);
    turnOffLightRelay(lightNum);
  }
}

// -------------------- Fan Control System Functions --------------------

void handleFanControlSystem() {
  // Read switch positions
  readSwitches();
  
  // Manual control has priority - check this first
  if (manualControl) {
    controlFansManually();
  } 
  // If no manual control but switch 5 is ON, use DHT11 automation
  else if (switchState5) {
    // Read temperature from DHT11
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();
    
    // Check if sensor reading is valid
    if (isnan(humidity) || isnan(temperature)) {
      Serial.println(F("Failed to read from DHT sensor!"));
      // If sensor fails, turn all fans off as a safety measure
      temperature = 34.0;
    } else {
      // Print sensor readings
      Serial.print(F("Humidity: "));
      Serial.print(humidity);
      Serial.print(F("%  Temperature: "));
      Serial.print(temperature);
      Serial.println(F("°C"));
      
      // Control fans based on temperature
      controlFansAutomatically(temperature);
    }
  }
  // Neither manual control nor automation enabled - turn all relays off
  else {
    turnAllFansOff();
  }
}

void readSwitches() {
  // Read switch states (LOW when switch is ON due to pullup resistors)
  switchState1 = !digitalRead(SWITCH_1);
  switchState2 = !digitalRead(SWITCH_2);
  switchState3 = !digitalRead(SWITCH_3);
  switchState4 = !digitalRead(SWITCH_4);
  switchState5 = !digitalRead(SWITCH_5);
  
  // Check if any manual override switch is on (switches 1-4)
  manualControl = switchState1 || switchState2 || switchState3 || switchState4;
  
  Serial.print("Fan control switch states: ");
  Serial.print(switchState1);
  Serial.print(", ");
  Serial.print(switchState2);
  Serial.print(", ");
  Serial.print(switchState3);
  Serial.print(", ");
  Serial.print(switchState4);
  Serial.print(", ");
  Serial.print(switchState5);
  Serial.print(" (DHT Automation: ");
  Serial.print(switchState5 ? "ON" : "OFF");
  Serial.println(")");
}

void controlFansManually() {
  // If switch 1 is on, turn on relay 1 (HIGH for active HIGH relay)
  if (switchState1) {
    digitalWrite(FAN_RELAY_1, HIGH);
    Serial.println("Manual mode: Fan Relay 1 ON");
  } else if (!switchState3) { // Only turn off if switch 3 is also off
    digitalWrite(FAN_RELAY_1, LOW);
  }
  
  // If switch 2 is on, turn on relay 2
  if (switchState2) {
    digitalWrite(FAN_RELAY_2, HIGH);
    Serial.println("Manual mode: Fan Relay 2 ON");
  } else if (!switchState3) { // Only turn off if switch 3 is also off
    digitalWrite(FAN_RELAY_2, LOW);
  }
  
  // If switch 3 is on, turn on relay 1 and relay 2
  if (switchState3) {
    digitalWrite(FAN_RELAY_1, HIGH);
    digitalWrite(FAN_RELAY_2, HIGH);
    Serial.println("Manual mode: Fan Relay 1 and 2 ON");
  }
  
  // If switch 4 is on, turn on relay 3
  if (switchState4) {
    digitalWrite(FAN_RELAY_3, HIGH);
    Serial.println("Manual mode: Fan Relay 3 ON");
  } else {
    digitalWrite(FAN_RELAY_3, LOW);
  }
}

void controlFansAutomatically(float temperature) {
  Serial.print("Automatic fan mode: Temperature = ");
  Serial.print(temperature);
  Serial.println("°C");
  
  // First turn all relays off
  digitalWrite(FAN_RELAY_1, LOW);
  digitalWrite(FAN_RELAY_2, LOW);
  digitalWrite(FAN_RELAY_3, LOW);
  
  // Then set according to temperature ranges
  if (temperature >= TEMP_VERY_HIGH) {
    // Temperature > 32°C: Turn on relay 3
    digitalWrite(FAN_RELAY_3, HIGH);
    Serial.println("Auto mode: Fan Relay 3 ON (temp > 32°C)");
  } 
  else if (temperature >= TEMP_HIGH) {
    // Temperature 27-32°C: Turn on relay 1 and 2
    digitalWrite(FAN_RELAY_1, HIGH);
    digitalWrite(FAN_RELAY_2, HIGH);
    Serial.println("Auto mode: Fan Relay 1 and 2 ON (temp 27-32°C)");
  } 
  else if (temperature >= TEMP_MEDIUM) {
    // Temperature 24-27°C: Turn on relay 2
    digitalWrite(FAN_RELAY_2, HIGH);
    Serial.println("Auto mode: Fan Relay 2 ON (temp 24-27°C)");
  } 
  else if (temperature < TEMP_LOW) {
    // Temperature < 23°C: Turn on relay 1
    digitalWrite(FAN_RELAY_1, HIGH);
    Serial.println("Auto mode: Fan Relay 1 ON (temp < 23°C)");
  }
  else {
    // If we're between 23-24°C, all relays remain off
    Serial.println("Auto mode: All fan relays OFF (temp 23-24°C)");
  }
}

void turnAllFansOff() {
  digitalWrite(FAN_RELAY_1, LOW);
  digitalWrite(FAN_RELAY_2, LOW);
  digitalWrite(FAN_RELAY_3, LOW);
  Serial.println("All fan relays OFF (automation disabled)");
}

// System status display function
void displaySystemStatus() {
  Serial.println("========== SYSTEM STATUS ==========");
  Serial.print("Network: ");
  Serial.print(WiFi.SSID());
  Serial.print(" | IP: ");
  Serial.println(WiFi.localIP());
  
  // Person detection status with camera details
  Serial.println("\n--- PERSON DETECTION STATUS ---");
  
  // Determine camera detection based on active lights
  bool updatedPersonDetectedCam1 = activeLights[0] || activeLights[1];
  bool updatedPersonDetectedCam2 = activeLights[2] || activeLights[3];
  
  if (updatedPersonDetectedCam1 || updatedPersonDetectedCam2) {
    // Display camera-specific detection
    for (int i = 0; i < 4; i++) {
      Serial.print("cam");
      Serial.print((i < 2) ? "1" : "2");  // First two lights use cam1, last two use cam2
      Serial.print(" light");
      Serial.print(i + 1);
      Serial.print(": ");
      
      bool camActive = (i < 2) ? updatedPersonDetectedCam1 : updatedPersonDetectedCam2;
      Serial.println(camActive ? "PERSON DETECTED" : "NO PERSON");
    }
  } else {
    Serial.println("No persons detected in any camera");
  }
  
  // Light status with lux values
  Serial.println("\n--- LIGHT & LUX SENSOR STATUS ---");
  for (int i = 0; i < 4; i++) {
    Serial.print("Light ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(activeLights[i] ? "ON" : "OFF");
    Serial.print(", Lux value - ");
    Serial.println(luxSensors[i].lastLuxValue);
  }
  
  // Fan status with temperature
  Serial.println("\n--- FAN & TEMPERATURE STATUS ---");
  // Read current temperature
  float temperature = dht.readTemperature();
  if (isnan(temperature)) {
    temperature = 34.0;  // Default value if reading fails
    Serial.println("Error reading temperature sensor!");
  }
  
  // Determine current fan state
  bool fanState = false;
  if (digitalRead(FAN_RELAY_1) == HIGH || 
      digitalRead(FAN_RELAY_2) == HIGH || 
      digitalRead(FAN_RELAY_3) == HIGH) {
    fanState = true;
  }
  
  Serial.print("Fan: ");
  Serial.print(fanState ? "ON" : "OFF");
  Serial.print(", Temperature: ");
  Serial.print(temperature);
  Serial.println("°C");
  
  Serial.println("==================================");
fb_upload();
}
void fb_upload() {
    delay(2000);

    if (Firebase.ready() && signupOk) {
        // Update Camera Detection Status
        bool updatedPersonDetectedCam1 = activeLights[0] || activeLights[1];
        bool updatedPersonDetectedCam2 = activeLights[2] || activeLights[3];

        Firebase.RTDB.setString(&fbdo, "/cam/cam1", updatedPersonDetectedCam1 ? "Detected" : "Not Detected");
        Firebase.RTDB.setString(&fbdo, "/cam/cam2", updatedPersonDetectedCam1 ? "Detected" : "Not Detected");
        Firebase.RTDB.setString(&fbdo, "/cam/cam3", updatedPersonDetectedCam2 ? "Detected" : "Not Detected");
        Firebase.RTDB.setString(&fbdo, "/cam/cam4", updatedPersonDetectedCam2 ? "Detected" : "Not Detected");

        // Curtain Status (assuming default state for now)
        Firebase.RTDB.setString(&fbdo, "/curtain", "Closed");

        // Get temperature and humidity readings
        float temperature = dht.readTemperature();
        float humidity = dht.readHumidity();

        if (isnan(temperature)) temperature = 34.0;
        if (isnan(humidity)) humidity = 50.0;

        // Check fan status (using the actual fan relay pins)
        bool fanState = (digitalRead(FAN_RELAY_1) == HIGH || 
                          digitalRead(FAN_RELAY_2) == HIGH || 
                          digitalRead(FAN_RELAY_3) == HIGH);

        // Create a fan object with both fan status and temperature
        Firebase.RTDB.setString(&fbdo, "/fan/fan", fanState ? "ON" : "OFF");
        Firebase.RTDB.setFloat(&fbdo, "/fan/temperature", temperature);

        // Light and Lux Sensor Data
        // Ensure we're using the exact path structure and data types expected by the React code
        for (int i = 0; i < 4; i++) {
            // Ensure light status is "ON" or "OFF" in uppercase to match React expectations
            Firebase.RTDB.setString(&fbdo, "/lights/light" + String(i + 1), 
                                  activeLights[i] ? "ON" : "OFF");
            
            // Convert lux values to strings to match React's expectation
            // React initializes lux values as strings: lux1: "1000", etc.
            Firebase.RTDB.setString(&fbdo, "/lights/lux" + String(i + 1), 
                                  String(luxSensors[i].lastLuxValue));
        }

        // Also add temperature and humidity at root level for other uses
        Firebase.RTDB.setFloat(&fbdo, "/temperature", temperature);
        Firebase.RTDB.setFloat(&fbdo, "/humidity", humidity);

        Serial.println("Firebase update completed.");
    } else {
        Serial.println("Firebase not ready.");
    }
}