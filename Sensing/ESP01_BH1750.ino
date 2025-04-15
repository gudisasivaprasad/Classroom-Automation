#include <ESP8266WiFi.h>  // For ESP-01 (ESP8266 based)
#include <WiFiUdp.h>
#include <Wire.h>
#include <BH1750.h>

// -------------------- WiFi Credentials --------------------
const char* ssid     = "Siva";
const char* password = "123456789";

// -------------------- UDP Configuration -------------------
WiFiUDP udp;
unsigned int localUdpPort = 4242;  // Make sure this matches the port that master ESP32 sends to
char incomingPacket[255];

// -------------------- BH1750 Setup ------------------------
BH1750 lightMeter;  // Using default I2C address 0x23 or 0x5C

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
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  
  // -------------------- Start UDP ---------------------------
  udp.begin(localUdpPort);
  Serial.printf("UDP listening on port %d\n", localUdpPort);
  
  // -------------------- Initialize I2C & BH1750 -------------
  // On ESP8266, the Wire.begin() format is Wire.begin(SDA, SCL).
  // We want SDA=GPIO2, SCL=GPIO0.
  Wire.begin(2, 0);
  
  // Initialize the BH1750 sensor
  if (lightMeter.begin(BH1750::ONE_TIME_HIGH_RES_MODE)) {
    Serial.println("BH1750 sensor initialized.");
  } else {
    Serial.println("Error initializing BH1750 sensor!");
  }
}

void loop() {
  // -------------------- Check for UDP Packets ---------------
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0'; // Null-terminate string
    }
    
    String request = String(incomingPacket);
    Serial.print("UDP packet contents: ");
    Serial.println(request);
    
    if (request == "REQUEST_LUX") {
      // Force sensor to make a new measurement in ONE_TIME mode
      lightMeter.configure(BH1750::ONE_TIME_HIGH_RES_MODE);
      
      // Wait for measurement to complete (typically ~120ms)
      delay(120);
      
      // Read Lux from BH1750
      float lux = lightMeter.readLightLevel();
      
      Serial.print("Lux: ");
      Serial.println(lux);
      
      // Convert lux reading to string - ensure it's an integer to match ESP32 expectations
      String luxString = String(int(lux));
      
      // Send response back to the requestor (ESP32 master)
      udp.beginPacket(udp.remoteIP(), udp.remotePort());
      udp.write(luxString.c_str());
      udp.endPacket();
      
      Serial.printf("Sent lux value %s to %s:%d\n", 
                   luxString.c_str(), 
                   udp.remoteIP().toString().c_str(), 
                   udp.remotePort());
    }
  }
  
  // Small delay for stability
  delay(100);
}