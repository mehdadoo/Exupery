#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "IgnitionSwitch.h"
#include "VoltageSensor.h"
#include "SpeedSensor.h"
#include "PedalSensor.h"
#include "Dashboard.h"
#include "InclinationSensor.h"
#include "BrakeSystem.h"
#include "ThrottleSystem.h"
#include "SteeringSystem.h"
#include "LCDDisplay.h"
#include "Buzzer.h"
#include "Horn.h"


IgnitionSwitch ignitionSwitch;
PortExpander& portExpander = PortExpander::getInstance();
VoltageSensor voltageSensor;
SpeedSensor speedSensor;
PedalSensor pedalSensor;
Dashboard dashboard(voltageSensor, speedSensor);
InclinationSensor inclinationSensor;
BrakeSystem brakeSystem(dashboard, speedSensor);
ThrottleSystem throttleSystem(dashboard, pedalSensor, speedSensor);
SteeringSystem steeringSystem(dashboard);
LCDDisplay lcdDisplay;
Buzzer& buzzer = Buzzer::getInstance();
Horn& horn = Horn::getInstance();

void setup()
{
  // Set event listeners
  dashboard.onRequestWiFi([]() {                  WiFiPrinter::setup();     });
  ignitionSwitch.setOnTurnedOnListener([]() {     start();                  });
  ignitionSwitch.setOnTurnedOffListener([]() {    shutdown();               });
  ignitionSwitch.setup();

   
}

void loop()
{
  ignitionSwitch.update();
  
  portExpander.update();
  speedSensor.update();
  pedalSensor.update();
  voltageSensor.update();
  dashboard.update();
  inclinationSensor.update();
  brakeSystem.update();
  throttleSystem.update();
  steeringSystem.update();
  buzzer.update();
  horn.update();

  lcdDisplay.update(dashboard.toggleState[0], !dashboard.buttonState[1], dashboard.toggleState[2], dashboard.toggleState[3], 
                          speedSensor.getSpeed(), pedalSensor.isStopped(), 
                          dashboard.joystick_throttle, dashboard.joystick_knob,  dashboard.joystick_steering,
                          voltageSensor.voltage,
                          steeringSystem.steering_percentage, brakeSystem.brakePercentage, throttleSystem.throttle1_percentage, throttleSystem.throttle2_percentage,
                          inclinationSensor.getInclinationAngle() );
  
  WiFiPrinterUpdate();
}

void start()
{
  lcdDisplay.start();
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  
  portExpander.start();
  speedSensor.start();
  pedalSensor.start();
  voltageSensor.start();
  dashboard.start();
  inclinationSensor.start();
  brakeSystem.start();
  throttleSystem.start();
  steeringSystem.start();

  WiFiPrinter::setup();
}

void shutdown()
{
  lcdDisplay.shutdown();
  
  steeringSystem.shutdown();
  throttleSystem.shutdown();
  brakeSystem.shutdown();
  inclinationSensor.shutdown();
  dashboard.shutdown();
  voltageSensor.shutdown();
  pedalSensor.shutdown();
  speedSensor.shutdown();
  portExpander.shutdown();
  
  SPI.end();
  Wire.end();
}

void WiFiPrinterUpdate()
{
  static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
  unsigned long currentTime = millis();

  // Check if enough time has been passed since last print call
  if (currentTime - lastUpdateTime >= UPDATE_OVER_HTTP_FREQUENCY) 
  {
      lastUpdateTime = currentTime;
      WiFiPrinter::printAll( ignitionSwitch.isKeyOn,
                          dashboard.toggleState[0], dashboard.toggleState[1], dashboard.toggleState[2], dashboard.toggleState[3], 
                          speedSensor.getSpeed(), pedalSensor.isStopped(),
                          dashboard.joystick_throttle, dashboard.joystick_knob,  dashboard.joystick_steering,
                          voltageSensor.voltage,
                          voltageSensor.batteryPercentage,
                          inclinationSensor.getInclinationAngle() );
  }
  WiFiPrinter::update();
}