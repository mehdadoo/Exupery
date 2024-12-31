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
      static void print(const String& value);
      static void printAll(int rpm, float speed, int brakeLeverPosition, int brakeServoPosition);

  private:
      static WebServer server;
      static unsigned long lastRetryTime; // Tracks the last retry attempt
      static int retryCount;             // Tracks the number of retries
};

#endif
