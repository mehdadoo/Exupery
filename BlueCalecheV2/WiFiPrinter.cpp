#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include "WebPage.h"
#include <ArduinoJson.h>
#include <WiFi.h>
#include <ArduinoOTA.h>

WebServer WiFiPrinter::server(80);
WebSocketsServer WiFiPrinter::webSocket(81);
bool WiFiPrinter::apStarted = false;
std::function<void(const String&)> WiFiPrinter::messageCallback = nullptr;

void WiFiPrinter::setup()
{
  if (apStarted)
    return;

  Serial.println("Starting WiFi Access Point...");
  apStarted = WiFi.softAP(AP_SSID, AP_PASSWORD);

  if (apStarted)
  {
    server.on("/", HTTP_GET, []() {
      server.send_P(200, "text/html", INDEX_HTML);
    });
    server.begin();

    webSocket.begin();
    webSocket.onEvent([](uint8_t num, WStype_t type, uint8_t* payload, size_t length) {
      if (type == WStype_TEXT && messageCallback)
      {
        String message;
        message.reserve(length);
        message.concat(reinterpret_cast<const char*>(payload), length);
        messageCallback(message);
      }
    });

    setupOTA();

    Serial.println("Access Point started");
    Serial.print("SSID: ");
    Serial.println(AP_SSID);
    Serial.print("IP Address: ");
    Serial.println(WiFi.softAPIP());
  }
  else
  {
    Serial.println("Failed to start Access Point");
  }

  print("Blue Caleche, Bonjour!");
}

void WiFiPrinter::onMessage(std::function<void(const String&)> callback)
{
  messageCallback = callback;
}

void WiFiPrinter::setupOTA()
{
  ArduinoOTA.setHostname(OTA_HOSTNAME);
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

void WiFiPrinter::printAll(int rpm, float speed, int brakeLeverPosition,
                           int brakeLeverRawValue, int frontServoPosition,
                           int backServoPosition, bool handBrakeEnabled,
                           bool nightLightsOn, bool hornOn)
{
  if (!apStarted)
    return;

  StaticJsonDocument<512> doc;
  doc["rpm"] = rpm;
  doc["speed"] = speed;
  doc["brakeLeverPosition"] = brakeLeverPosition;
  doc["brakeLeverRawValue"] = brakeLeverRawValue;
  doc["frontServoPosition"] = frontServoPosition;
  doc["backServoPosition"] = backServoPosition;
  doc["handBrakeEnabled"] = handBrakeEnabled;
  doc["nightLightsOn"] = nightLightsOn;
  doc["hornOn"] = hornOn;

  String json;
  serializeJson(doc, json);
  webSocket.broadcastTXT(json);
}
