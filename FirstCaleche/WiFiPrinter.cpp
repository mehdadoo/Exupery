#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include <ArduinoJson.h>

// Initialize static member
WebServer WiFiPrinter::server(80);  // Set up the server on port 80
String printMessage = "{}";  // Initial JSON message (empty)

// Static method to begin Wi-Fi connection and start the server
void WiFiPrinter::setup() 
{
  #ifndef DEBUG_MODE
    return;
  #endif

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


      // Remove the "message" field
      DynamicJsonDocument doc(1024); // Allocate enough space for the document
      deserializeJson(doc, printMessage);  // Deserialize the printMessage JSON
      if (doc.containsKey("message")) 
        doc.remove("message");
      serializeJson(doc, printMessage);// Serialize the updated JSON document back into the printMessage string
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
void WiFiPrinter::update()
{
  if (WiFi.status() == WL_CONNECTED) 
  {
    server.handleClient(); // Handle HTTP requests
    return;
  }
  // Handle Wi-Fi connection retry logic if disconnected (optional)
}

void WiFiPrinter::print(const String& value) {
  // Create a dynamic JSON document to parse and update the existing JSON
  DynamicJsonDocument doc(1024); // Allocate enough space for the document

  // Deserialize the current JSON string (printMessage) into the document
  deserializeJson(doc, printMessage);

  // Append the new message to the existing one with a newline character
  if (doc.containsKey("message")) {
    String currentMessage = doc["message"].as<String>();
    currentMessage += "\n" + value;  // Add the new message with a newline
    doc["message"] = currentMessage;  // Update the "message" field
  } else {
    doc["message"] = value;  // If no message field exists, create it
  }

  // Serialize the updated JSON document back into the printMessage string
  serializeJson(doc, printMessage);

  // Print to Serial for testing
  Serial.println(printMessage);
}

// Overloaded print method to handle integer values
void WiFiPrinter::printAll(int rpm, float speed, int brakeLeverPosition, int brakeServoPosition, int gear, int gearboxServoPosition)
{
  // Create a dynamic JSON document to parse and update the existing JSON
  DynamicJsonDocument doc(1024); // Allocate enough space for the document

  // Deserialize the current JSON string (printMessage) into the document
  deserializeJson(doc, printMessage);

  // Update the value based on the dataType
  doc["rpm"] = rpm; 
  doc["speed"] = speed; 
  doc["brakeLeverPosition"] = brakeLeverPosition;
  doc["brakeServoPosition"] = brakeServoPosition;
  doc["gear"] = gear;
  doc["gearboxServoPosition"] = gearboxServoPosition;

  // Serialize the updated JSON document back into the printMessage string
  serializeJson(doc, printMessage);
}
