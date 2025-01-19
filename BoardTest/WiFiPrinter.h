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
      static void printAll(bool powerSwitch,
                          bool button1, bool button2, bool button3, bool button4, 
                          bool speedSensor, bool pedalSensor,
                          int joystick_throttle, int joystick_knob, int joystick_steering,
                          float voltage,
                          float current,
                          float inclinationAngle);

  private:
      static void setupOTA();
      static WebServer server;
      static unsigned long lastRetryTime; // Tracks the last retry attempt
      static int retryCount;             // Tracks the number of retries
};

#endif
