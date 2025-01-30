#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>

#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "IgnitionSwitch.h"
#include "VoltageSensor.h"
#include "InclinationSensor.h"
#include "SpeedSensor.h"
#include "PedalSensor.h"
#include "BrakeSystem.h"
#include "ThrottleSystem.h"
#include "Dashboard.h"
#include "SteeringSystem.h"

PortExpander& portExpander = PortExpander::getInstance();//MPC23S17 port expander
IgnitionSwitch ignitionSwitch;
InclinationSensor inclinationSensor;
VoltageSensor voltageSensor;//ADS1115 current and voltage reader
SpeedSensor speedSensor;
PedalSensor pedalSensor;
Dashboard dashboard(voltageSensor);
BrakeSystem brakeSystem(dashboard, speedSensor); // Pass speedSensor to the constructor
ThrottleSystem throttleSystem(dashboard, pedalSensor);
SteeringSystem steeringSystem(dashboard);

void setup()
{
  WiFiPrinter::setup();

  shutdown();

  // Set event listeners
  ignitionSwitch.setOnTurnedOnListener([]() {     start();        });
  ignitionSwitch.setOnTurnedOffListener([]() {    shutdown();     });
  ignitionSwitch.setup();

 
}

void loop()
{
  ignitionSwitch.update();
  
  inclinationSensor.update();
  portExpander.update();
  speedSensor.update();
  pedalSensor.update();
  voltageSensor.update();
  dashboard.update();
  brakeSystem.update();
  throttleSystem.update();
  steeringSystem.update();
  
  WiFiPrinterUpdate();
}

void start()
{
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  
  portExpander.start();
  speedSensor.start();
  pedalSensor.start();
  voltageSensor.start();
  dashboard.start();
  brakeSystem.start();
  throttleSystem.start();
  steeringSystem.start();
  inclinationSensor.start();
}

void shutdown()
{
  inclinationSensor.shutdown();
  steeringSystem.shutdown();
  throttleSystem.shutdown();
  brakeSystem.shutdown();
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
      lastUpdateTime = currentTime; // Update the last update time
      WiFiPrinter::printAll( ignitionSwitch.isKeyOn,
                          dashboard.toggleState[0], dashboard.toggleState[1], dashboard.toggleState[2], dashboard.toggleState[3], 
                          speedSensor.getSpeed(), steeringSystem.servoValue,
                          dashboard.joystick_throttle, dashboard.joystick_knob,  dashboard.joystick_steering,
                          voltageSensor.voltage,
                          voltageSensor.batteryPercentage,
                          inclinationSensor.getInclinationAngle() );
  }
  WiFiPrinter::update();
}