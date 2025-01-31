#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "Buzzer.h"


// Initialize static member
WebServer WiFiPrinter::server(80);  // Set up the server on port 80
String printMessage = "{}";  // Initial JSON message (empty)

// Static method to begin Wi-Fi connection and start the server
void WiFiPrinter::setup() 
{
  if (WiFi.status() == WL_CONNECTED) 
    return;

  Serial.println("Connecting to WiFi...");

  int retryCount = 0;
  while (retryCount < MAX_WIFI_CONNECTION_RETRIES) 
  {
      unsigned long startMillis = millis(); // Track start time for timeout
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

      while (WiFi.status() != WL_CONNECTED && millis() - startMillis < RETRY_INTERVAL)
          delay(10);

      if (WiFi.status() == WL_CONNECTED) 
          break; // Exit loop if connected

      retryCount++; // Increase retry counter
      Buzzer::getInstance().toggle();
  }


  if (WiFi.status() == WL_CONNECTED) 
  {
    // Initialize web server
    server.on("/getData", HTTP_GET, []() 
    {
      // Allow cross-origin requests
      server.sendHeader("Access-Control-Allow-Origin", "*");
      server.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
      server.sendHeader("Access-Control-Allow-Headers", "Content-Type, X-Requested-With");
      
      // Send the current message as raw JSON
      server.send(200, "application/json", printMessage);


      // Remove the "message" field
      DynamicJsonDocument doc(1024); // Allocate enough space for the document
      deserializeJson(doc, printMessage);  // Deserialize the printMessage JSON
      if (doc.containsKey("message")) 
        doc.remove("message");
      serializeJson(doc, printMessage);// Serialize the updated JSON document back into the printMessage string
    });

    server.begin();

    setupOTA();

    Serial.println("Connected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Buzzer::getInstance().off();
  } 
  else 
  {
    Serial.println("Failed to connect to WiFi");

    Buzzer::getInstance().beep3();
  }

  print("Blue Calèche, Bonjour!");
}

void WiFiPrinter::setupOTA() 
{
  ArduinoOTA.begin();
}

// Static method to handle Wi-Fi retry logic and keep server running
void WiFiPrinter::update()
{
  if (WiFi.status() == WL_CONNECTED) 
  {
    server.handleClient(); // Handle HTTP requests
    ArduinoOTA.handle(); // Handle OTA updates
  }
}

void WiFiPrinter::print(const String& value) 
{
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
  //Serial.println(printMessage);
}

// Overloaded print method to handle integer values
void WiFiPrinter::printAll(bool powerSwitch,
                          bool button1, bool button2, bool button3, bool button4, 
                          int speedSensor, int pedalSensor,
                          int joystick_throttle, int joystick_knob, int joystick_steering,
                          float voltage,
                          float current,
                          float inclinationAngle
                          )
{
  if (WiFi.status() != WL_CONNECTED) 
    return;

  // Create a dynamic JSON document to parse and update the existing JSON
  DynamicJsonDocument doc(1024); // Allocate enough space for the document

  // Deserialize the current JSON string (printMessage) into the document
  deserializeJson(doc, printMessage);

  // Update the value based on the dataType
  doc["powerSwitch"] = powerSwitch; 
  doc["button1"] = button1; 
  doc["button2"] = button2; 
  doc["button3"] = button3; 
  doc["button4"] = button4; 
  doc["speedSensor"] = speedSensor;
  doc["pedalSensor"] = pedalSensor;
  doc["joystick_throttle"] = joystick_throttle;
  doc["joystick_knob"] = joystick_knob;
  doc["joystick_steering"] = joystick_steering;
  doc["voltage"] = voltage;
  doc["current"] = current;
  doc["inclinationAngle"] = inclinationAngle;

  // Serialize the updated JSON document back into the printMessage string
  serializeJson(doc, printMessage);
}
