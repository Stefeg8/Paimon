#include <ESP8266WiFi.h>
//We can use either this one or ESP32WiFi

const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";
const char* host = "192.168.1.100";  // IP of remote computer
const int port = 8080;               // Port to connect to

void setup() {
  Serial.begin(115200);
  
  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  
  Serial.println("Connected to WiFi");

  // Connect to the server
  WiFiClient client;
  if (!client.connect(host, port)) {
    Serial.println("Connection failed");
    return;
  }

  // Send data to the remote computer
  client.println("Hello from Teensy");
  
  // Receive data from the remote computer
  while (client.available()) {
    String response = client.readStringUntil('\n');
    Serial.println("Received: " + response);
  }
  
  client.stop();
}

void loop() {
  // Keep connection alive or perform other tasks
}
