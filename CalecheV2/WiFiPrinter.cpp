#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include "WebPage.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "Buzzer.h"

WebServer WiFiPrinter::server(80);
WebSocketsServer WiFiPrinter::webSocket(81);
bool WiFiPrinter::apStarted = false;

void WiFiPrinter::setup()
{
  if (apStarted)
    return;

  Serial.println("Starting WiFi Access Point...");
  Buzzer::getInstance().beep();

  apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);

  if (apStarted)
  {
    server.on("/", HTTP_GET, []() {
      server.send_P(200, "text/html", INDEX_HTML);
    });
    server.begin();
    webSocket.begin();
    setupOTA();

    Serial.println("Access Point started");
    Serial.print("SSID: ");
    Serial.println(AP_SSID);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());

    Buzzer::getInstance().off();
    Buzzer::getInstance().beep();
  }
  else
  {
    Serial.println("Failed to start Access Point");
    Buzzer::getInstance().beep3();
  }

  print("Blue Calèche, Bonjour!");
}

void WiFiPrinter::setupOTA()
{
  ArduinoOTA.begin();
}

void WiFiPrinter::update()
{
  if (apStarted)
  {
    server.handleClient();
    webSocket.loop();
    ArduinoOTA.handle();
  }
}

void WiFiPrinter::print(const String& value)
{
  if (!apStarted)
    return;

  StaticJsonDocument<256> doc;
  doc["message"] = value;
  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}

void WiFiPrinter::printAll(bool powerSwitch,
                           bool button1, bool button2, bool button3, bool button4,
                           int speedSensor, int pedalSensor,
                           int joystick_throttle, int joystick_knob, int joystick_steering,
                           float voltage,
                           float inclinationAngle)
{
  if (!apStarted)
    return;

  StaticJsonDocument<512> doc;
  doc["powerSwitch"]       = powerSwitch;
  doc["button1"]           = button1;
  doc["button2"]           = button2;
  doc["button3"]           = button3;
  doc["button4"]           = button4;
  doc["speedSensor"]       = speedSensor;
  doc["pedalSensor"]       = pedalSensor;
  doc["joystick_throttle"] = joystick_throttle;
  doc["joystick_knob"]     = joystick_knob;
  doc["joystick_steering"] = joystick_steering;
  doc["voltage"]           = voltage;

  doc["inclinationAngle"]  = inclinationAngle;

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}
