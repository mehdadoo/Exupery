#ifndef WiFiPrinter_h
#define WiFiPrinter_h

#include <WebServer.h>
#include <WebSocketsServer.h>
#include <functional>

class WiFiPrinter 
{
  public:
      static void setup();
      static void update();
      static void print(const String& value);
      static void printAll(int rpm, float speed, int brakeLeverPosition,
                           int brakeLeverRawValue, int frontServoPosition,
                           int backServoPosition, bool handBrakeEnabled,
                           bool nightLightsOn, bool hornOn);
      static void onMessage(std::function<void(const String&)> callback);

  private:
      static void setupOTA();
      static WebServer server;
      static WebSocketsServer webSocket;
      static bool apStarted;
      static std::function<void(const String&)> messageCallback;
};

#endif
