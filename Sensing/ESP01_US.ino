#include <ESP8266WiFi.h>
#include <WiFiUdp.h>

// -------------------- WiFi Credentials --------------------
const char* ssid = "Siva";
const char* password = "123456789";

// -------------------- UDP Configuration -------------------
WiFiUDP udp;
const int slaveUdpPort = 4220;         // UDP port of the Slave ESP01
const char* masterIp = "192.168.96.226"; // IP of the Master ESP32
const int masterUdpPort = 4210;        // Master UDP port
char incomingPacket[255];              // Buffer for incoming packets

// -------------------- Ultrasonic Sensor Pins -------------------
const int trigPin = 2; // GPIO2 (D4)
const int echoPin = 0; // GPIO0 (D3)

// -------------------- Threshold for Person Detection -------------------
const int distanceThreshold = 50; // Distance threshold in cm

void setup() {
  Serial.begin(115200);

  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
  Serial.print("ESP01 IP Address: ");
  Serial.println(WiFi.localIP());

  // Initialize UDP
  udp.begin(slaveUdpPort);
  Serial.printf("Slave listening on UDP port %d\n", slaveUdpPort);

  // Initialize ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Listen for messages from the Master ESP32
  int packetSize = udp.parsePacket();
  if (packetSize > 0) {
    int len = udp.read(incomingPacket, sizeof(incomingPacket) - 1);
    if (len > 0) {
      incomingPacket[len] = '\0'; // Null-terminate the received data
    }

    String request = String(incomingPacket);
    Serial.printf("Message from Master: %s\n", request.c_str());

    if (request == "REQUEST_ULTRASONIC") {
      int distance = getDistance();
      if (distance > 0 && distance < distanceThreshold) {
        sendToMaster("PERSON_DETECTED");
      } else {
        sendToMaster("NO_PERSON");
      }
    }
  }

  delay(100); // Short delay for UDP communication
}

// Function to measure distance using ultrasonic sensor
int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  int distance = duration * 0.034 / 2; // Convert to cm
  Serial.printf("Measured Distance: %d cm\n", distance);
  return distance;
}

// Function to send a message to the Master ESP32
void sendToMaster(const char* message) {
  udp.beginPacket(masterIp, masterUdpPort);
  udp.write((uint8_t*)message, strlen(message)); // Cast and specify the length
  udp.endPacket();
  Serial.printf("Message sent to Master: %s\n", message);
}
