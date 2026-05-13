#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include "Buzzer.h"

WebServer WiFiPrinter::server(80);
WebSocketsServer WiFiPrinter::webSocket(81);

void WiFiPrinter::setup()
{
  if (WiFi.status() == WL_CONNECTED)
    return;

  Serial.println("Connecting to WiFi...");

  int retryCount = 0;

  while (retryCount < MAX_WIFI_CONNECTION_RETRIES)
  {
    unsigned long startMillis = millis();
    Buzzer::getInstance().beep();

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    while (WiFi.status() != WL_CONNECTED && millis() - startMillis < RETRY_INTERVAL)
    {
      delay(10);
      if (millis() - startMillis >= 50)
        Buzzer::getInstance().off();
    }

    if (WiFi.status() == WL_CONNECTED)
      break;

    retryCount++;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    server.begin();
    webSocket.begin();
    setupOTA();

    Serial.println("Connected to Wi-Fi");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());

    Buzzer::getInstance().off();
    Buzzer::getInstance().beep();
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

void WiFiPrinter::update()
{
  if (WiFi.status() == WL_CONNECTED)
  {
    server.handleClient();
    webSocket.loop();
    ArduinoOTA.handle();
  }
}

void WiFiPrinter::print(const String& value)
{
  if (WiFi.status() != WL_CONNECTED)
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
  if (WiFi.status() != WL_CONNECTED)
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
