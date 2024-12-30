#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include <ArduinoJson.h>

// Initialize static member
WebServer WiFiPrinter::server(80);  // Set up the server on port 80
String printMessage = "{}";  // Initial JSON message (empty)

// Static method to begin Wi-Fi connection and start the server
void WiFiPrinter::setup() {
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("Connecting to WiFi...");

  unsigned long startMillis = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startMillis < 5000) {
    delay(10);
  }

  if (WiFi.status() == WL_CONNECTED) {
    // Initialize web server
    server.on("/getData", HTTP_GET, []() {
      // Allow cross-origin requests
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
      server.sendHeader("Access-Control-Allow-Headers", "Content-Type, X-Requested-With");
      
      // Send the current message as raw JSON
      Serial.println("Request received. Sending data...");
      server.send(200, "application/json", printMessage);
    });
    server.begin();

    Serial.println("Connected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("Failed to connect to WiFi");
  }
}

// Static method to handle Wi-Fi retry logic and keep server running
void WiFiPrinter::update() {
  if (WiFi.status() == WL_CONNECTED) {
    server.handleClient(); // Handle HTTP requests
    return;
  }
  // Handle Wi-Fi connection retry logic if disconnected (optional)
}

// Method to update the JSON string or create new key-value pairs
void WiFiPrinter::print(int dataType, const String& value) {
  // Create a dynamic JSON document to parse and update the existing JSON
  DynamicJsonDocument doc(1024); // Allocate enough space for the document

  // Deserialize the current JSON string (printMessage) into the document
  deserializeJson(doc, printMessage);

  doc["message"] = value;  // Update or add the "rpm" field

  // Serialize the updated JSON document back into the printMessage string
  serializeJson(doc, printMessage);

  // Print to Serial for testing
  Serial.println(printMessage);
}

// Overloaded print method to handle integer values
void WiFiPrinter::print(int dataType, int value) {
  // Create a dynamic JSON document to parse and update the existing JSON
  DynamicJsonDocument doc(1024); // Allocate enough space for the document

  // Deserialize the current JSON string (printMessage) into the document
  deserializeJson(doc, printMessage);

  // Update the value based on the dataType
  if (dataType == RPM_CONSTANT) {
    doc["rpm"] = value;  // Update or add the "rpm" field
  } 
  else if (dataType == SPEED_CONSTANT) {
    doc["speed"] = value;  // Update or add the "speed" field
  }
  else if (dataType == BRAKE_LEVER_CONSTANT) {
    doc["brakeLeverPosition"] = value;  // Update or add the "speed" field
  }
  else if (dataType == BRAKE_SERVO_CONSTANT) {
    doc["brakeServoPosition"] = value;  // Update or add the "speed" field
  }

  // Serialize the updated JSON document back into the printMessage string
  serializeJson(doc, printMessage);

  // Print to Serial for testing
  Serial.println(printMessage);
}
