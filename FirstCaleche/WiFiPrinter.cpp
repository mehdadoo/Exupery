#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"

// Initialize static member
WebServer WiFiPrinter::server(80);  // Set up the server on port 80
unsigned long WiFiPrinter::lastRetryTime = 0; // Initialize to 0
int WiFiPrinter::retryCount = 0;              // Initialize to 0
String printMessage = "";  // Global variable to store the message to be printed

// Static method to begin Wi-Fi connection and start the server
void WiFiPrinter::setup()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    if (WiFi.status() == WL_CONNECTED)
    {
        // Initialize web server
        server.on("/getData", HTTP_GET, []() {
            server.send(200, "text/plain", printMessage); // Send the current message
        });
        server.begin();
        retryCount = 0; // Reset retry count upon successful connection
    }
}


void WiFiPrinter::update() 
{       
  if (WiFi.status() == WL_CONNECTED)
  {
      server.handleClient(); // Handle HTTP requests
      return;
  }

  // Check if retry attempts are exhausted
  if (retryCount >= MAX_WIFI_CONNECTION_RETRIES)
  {
      return;
  }

  // Check if it's time to retry
  unsigned long currentMillis = millis();
  if (currentMillis - lastRetryTime >= RETRY_INTERVAL)
  {
      lastRetryTime = currentMillis;
      retryCount++;
      setup(); // Retry WiFi setup
  }
}

// Static method to send data over Wi-Fi to connected clients
void WiFiPrinter::print(int dataType, const String& value) 
{
  if (WiFi.status() != WL_CONNECTED)
    return;

  String jsonData = "{";

  // Handle different data types
  if (dataType == RPM_CONSTANT) {
    jsonData += "\"RPM\": \"" + value + "\"";
  } 
  else if (dataType == SPEED_CONSTANT) {
    jsonData += "\"speed\": \"" + value + "\"";
  }
  else if (dataType == BRAKE_LEVER_CONSTANT) {
    jsonData += "\"brakeLeverPosition\": \"" + value + "\"";
  }
  else if (dataType == BRAKE_SERVO_CONSTANT) {
    jsonData += "\"brakeServoPosition\": \"" + value + "\"";
  }
  else if (dataType == CUSTOM_MESSAGE) {
    jsonData += "\"message\": \"" + value + "\"";
  }

  // Closing the JSON object
  jsonData += "}";

  // Send the JSON data over Wi-Fi to the client
  WiFiClient client = server.client();  // Get the connected client
  if (client && client.connected()) 
  {
    client.println(jsonData);  // Send the JSON data to the client
  }

}

// Overloaded print method to handle integer values
void WiFiPrinter::print(int dataType, int value)
{
  if (WiFi.status() != WL_CONNECTED)
    return;
    
  String jsonData = "{";

  // Handle different data types
  if (dataType == RPM_CONSTANT) {
    jsonData += "\"RPM\": " + String(value);
  } 
  else if (dataType == SPEED_CONSTANT) {
    jsonData += "\"speed\": " + String(value);
  }
  else if (dataType == BRAKE_LEVER_CONSTANT) {
    jsonData += "\"brakeLeverPosition\": " + String(value);
  }
  else if (dataType == BRAKE_SERVO_CONSTANT) {
    jsonData += "\"brakeServoPosition\": \"" + String(value);
  }
  else if (dataType == CUSTOM_MESSAGE) {
    jsonData += "\"message\": \"" + String(value) + "\"";
  }

  // Closing the JSON object
  jsonData += "}";

  // Send the JSON data over Wi-Fi to the client
  WiFiClient client = server.client();  // Get the connected client
  if (client && client.connected()) 
  {
    client.println(jsonData);  // Send the JSON data to the client
  }
}
