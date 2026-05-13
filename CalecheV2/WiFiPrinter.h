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
    static void printAll(bool powerSwitch,
                        bool button1, bool button2, bool button3, bool button4,
                        int speedSensor, int pedalSensor,
                        int joystick_throttle, int joystick_knob, int joystick_steering,
                        float voltage,
                        float inclinationAngle);
    static void onMessage(std::function<void(const String&)> callback);

  private:
    static void setupOTA();
    static WebServer server;
    static WebSocketsServer webSocket;
    static bool apStarted;
    static std::function<void(const String&)> messageCallback;
};

#endif
