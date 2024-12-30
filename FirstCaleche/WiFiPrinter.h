#ifndef WiFiPrinter_h
#define WiFiPrinter_h

#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

class WiFiPrinter 
{
  public:
      static void setup();
      static void update();
      static void print(int dataType, const String& value);
      static void print(int dataType, int value);  // Overloaded function for integers

  private:
      static WebServer server;
      static unsigned long lastRetryTime; // Tracks the last retry attempt
      static int retryCount;             // Tracks the number of retries
};

#endif
