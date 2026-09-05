#include "PinDefinitions.h"
#include "BrakeSystem.h"
#include "NightLights.h"
#include "Horn.h"
#include "WiFiPrinter.h"
#include <ESP32Servo.h>

//#define DEBUG_MODE


SpeedSensor speedSensor;              // Create the SpeedSensor instance
BrakeSystem brakeSystem(speedSensor); // Pass speedSensor to the constructor
NightLights nightLights; // Pass speedSensor to the  onstructor


void setup()
{
  setupHeartbeat();
  setupSerial();
  WiFiPrinter::setup();
  speedSensor.setup();
  brakeSystem.setup();
  nightLights.setup();
  Horn::getInstance().setup();

  WiFiPrinter::onMessage([](const String& message) {
    if (message == "nightButton")
      nightLights.toggle();
    else if (message == "hornButton")
      Horn::getInstance().beep();
    else if (message == "handBrakeButton")
      brakeSystem.toggleHandBrake();
    else if (message == "potPhysical")
      brakeSystem.usePhysicalBrakeLever();
    else if (message.startsWith("pot:"))
      brakeSystem.setSimulatedBrakeLeverPosition(message.substring(4).toInt());
  });

  WiFiPrinter::print("Setup Complete!");
}

void setHeartbeatLed(bool enabled)
{
  #ifdef RGB_BUILTIN
    rgbLedWrite(RGB_BUILTIN,
                0,
                enabled ? HEARTBEAT_BRIGHTNESS : 0,
                0);
  #else
    digitalWrite(LED_BUILTIN, enabled ? HIGH : LOW);
  #endif
}

void setupHeartbeat()
{
  #ifndef RGB_BUILTIN
    pinMode(LED_BUILTIN, OUTPUT);
  #endif

  setHeartbeatLed(false);
}

void updateHeartbeat()
{
  static unsigned long lastCycleStart = 0;
  static unsigned long phaseStart = 0;
  static uint8_t heartbeatPhase = 0;
  static bool ledOn = false;
  unsigned long currentTime = millis();

  if (heartbeatPhase == 0 &&
      currentTime - lastCycleStart >= HEARTBEAT_INTERVAL_MS)
  {
    lastCycleStart = currentTime;
    phaseStart = currentTime;
    heartbeatPhase = 1;
    ledOn = true;
    setHeartbeatLed(true);
  }
  else if (heartbeatPhase == 1 &&
           currentTime - phaseStart >= HEARTBEAT_DURATION_MS)
  {
    phaseStart = currentTime;
    heartbeatPhase = 2;
    ledOn = false;
    setHeartbeatLed(false);
  }
  else if (heartbeatPhase == 2 &&
           currentTime - phaseStart >= HEARTBEAT_GAP_MS)
  {
    phaseStart = currentTime;
    heartbeatPhase = 3;
    ledOn = true;
    setHeartbeatLed(true);
  }
  else if (heartbeatPhase == 3 &&
           currentTime - phaseStart >= HEARTBEAT_DURATION_MS)
  {
    heartbeatPhase = 0;
    ledOn = false;
    setHeartbeatLed(false);
  }

  digitalWrite(HAND_BRAKE_LED_PIN,
               (brakeSystem.handBrakeEnabled || ledOn) ? HIGH : LOW);
}

void setupSerial()
{
  #ifndef DEBUG_MODE
    return;
  #endif


  Serial.begin(9600);

  // Check if the serial port is available
  unsigned long startMillis = millis();
  while (!Serial && millis() - startMillis < 2000) 
  {
    // Wait up to 5 seconds for the serial connection
    delay(10);
  }
  Serial.println( "Serial startup: " + String ( millis() - startMillis ) );
  WiFiPrinter::print("First Calèche, Bonjour!");
}


void updateOverWebSocket()
{
  static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
  unsigned long currentTime = millis();

  // Check if enough time has been passed since last print call
  if (currentTime - lastUpdateTime >= UPDATE_OVER_WS_FREQUENCY) 
  {
      lastUpdateTime = currentTime; // Update the last update time
      WiFiPrinter::printAll( speedSensor.getRPM(), speedSensor.getSpeed(),
                             brakeSystem.brakeLeverPosition, brakeSystem.brakeLeverRawValue,
                             brakeSystem.frontServoPosition, brakeSystem.backServoPosition,
                             brakeSystem.handBrakeEnabled,
                             nightLights.isOn(), Horn::getInstance().isOn() );
  }

  WiFiPrinter::update();
}


void loop()
{
  speedSensor.update();
  brakeSystem.update();
  updateHeartbeat();
  nightLights.update();
  Horn::getInstance().update();

  updateOverWebSocket();
}
