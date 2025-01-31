#include "PedalSensor.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include <Arduino.h>


void PedalSensor::update() 
{
  PortExpander& portExpander = PortExpander::getInstance();

 if( !portExpander.initialized || !initialized)
    return;

  uint8_t sensorState = portExpander.digitalReadMCP23S17(PORT_EXPANDER_PORT_A, SENSOR_PEDAL_TRIGGER_PIN);  // Read current state of sensor

  if ( sensorState == HIGH )
    lastSensorTriggerTime = millis();


  if (millis() - lastSensorTriggerTime > PEDAL_SENSOR_STOP_DELAY) 
    stoppedState = true;
  else
     stoppedState = false;
}

void PedalSensor::shutdown() 
{
  if( initialized)
  {
  }
  initialized = false;
}

void PedalSensor::start() 
{
    initialized = true;
}
