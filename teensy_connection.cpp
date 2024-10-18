#include <Audio.h>
#include <Wire.h>
#include <ESP8266WiFi.h>  // Change to <WiFi.h> for ESP32

// Audio setup (for capturing mic input)
AudioInputI2S            i2s1;       // Input from I2S mic
AudioOutputI2S           i2s2;       // Output to speakers
AudioConnection          patchCord1(i2s1, 0, i2s2, 0);
AudioControlSGTL5000     sgtl5000;   // Control for Teensy Audio Shield

// Wi-Fi settings
const char* ssid = "your_SSID";          // Replace with your Wi-Fi SSID
const char* password = "your_PASSWORD";   // Replace with your Wi-Fi password

IPAddress server(192, 168, 1, 100);  // Change to the IP address of the server
const int port = 8080;           

WiFiClient client;

void setup() {
  Serial.begin(115200);
  AudioMemory(8);
  sgtl5000.enable();
  sgtl5000.volume(0.5);

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("\nConnected to Wi-Fi");

  // Attempt to connect to the server
  if (client.connect(server, port)) {
    Serial.println("Connected to server");
  } else {
    Serial.println("Connection to server failed");
  }
}

void loop() {
  // Capture audio and send it to the server
  int16_t audio_buffer[256];
  
  // Check if the server is still connected
  if (client.connected()) {
    // Read audio from I2S mic
    i2s1.read(audio_buffer, 256);
    
    // Send audio buffer to the server
    client.write((byte*)audio_buffer, sizeof(audio_buffer));

    // Receive audio response from the server and play it
    if (client.available()) {
      byte response[512];
      int bytesRead = client.read(response, 512);

      // Play received audio (this is just an example, adjust according to your setup)
      i2s2.write((int16_t*)response, bytesRead / 2);  // Assume 16-bit audio
    }
  } else {
    Serial.println("Disconnected from server");
    client.connect(server, port);  // Attempt to reconnect
  }
}
