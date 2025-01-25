#include <Arduino.h>

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
VoltageSensor voltageSensor;//ADS1115 current and voltage reader
IgnitionSwitch ignitionSwitch;
Dashboard dashboard;
InclinationSensor inclinationSensor;
SpeedSensor speedSensor;
PedalSensor pedalSensor;
BrakeSystem brakeSystem(dashboard, speedSensor); // Pass speedSensor to the constructor
ThrottleSystem throttleSystem(dashboard, pedalSensor);
SteeringSystem steeringSystem(dashboard);

void setup()
{
  initializeSerial();

  inclinationSensor.start();

  // Set event listeners
  ignitionSwitch.setOnTurnedOnListener([]() {
    start();
  });

  ignitionSwitch.setOnTurnedOffListener([]() {
      shutdown();
  });

  ignitionSwitch.setup();
}

void loop()
{
  ignitionSwitch.update();
  inclinationSensor.update();
  speedSensor.update();
  pedalSensor.update();
  voltageSensor.update();
  
  if( dashboard.initialized )
  {
    dashboard.batteryPercentage = voltageSensor.batteryPercentage;
    dashboard.update();
    brakeSystem.update();
    throttleSystem.update();
    steeringSystem.update();
  }
  
  updateOverHTTP();
}

void start()
{
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  portExpander.start();
  voltageSensor.start();
  dashboard.start();
  brakeSystem.start();
  throttleSystem.start();
  steeringSystem.start();
}

void shutdown()
{
  steeringSystem.shutdown();
  throttleSystem.shutdown();
  brakeSystem.shutdown();
  dashboard.shutdown();
  voltageSensor.shutdown();
  portExpander.shutdown();
  SPI.end();
}

void initializeSerial()
{
  Serial.begin(9600);
  WiFiPrinter::setup();
  WiFiPrinter::print("Blue Calèche, Bonjour!");
}

void updateOverHTTP()
{
  static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
  unsigned long currentTime = millis();

  // Check if enough time has been passed since last print call
  if (currentTime - lastUpdateTime >= UPDATE_OVER_HTTP_FREQUENCY) 
  {
      lastUpdateTime = currentTime; // Update the last update time
      WiFiPrinter::printAll( ignitionSwitch.isKeyOn,
                          dashboard.toggleState[0], pedalSensor.isStopped(), dashboard.buttonState[2], dashboard.buttonState[3], 
                          speedSensor.getSpeed(), speedSensor.getRPM(),
                          dashboard.joystick_throttle, dashboard.joystick_knob,  dashboard.joystick_steering,
                          voltageSensor.voltage,
                          voltageSensor.current,
                          inclinationSensor.getInclinationAngle() );
  }

  WiFiPrinter::update();
}