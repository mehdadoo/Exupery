#include "PedalSensor.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include <Arduino.h>


void PedalSensor::update() 
{
  PortExpander& portExpander = PortExpander::getInstance();

  if (!portExpander.initialized)
    return;

  uint8_t sensorState = portExpander.digitalRead(SENSOR_PEDAL_TRIGGER_PIN);  // Read current state of sensor
  static int lastSensorState = LOW;
  static int sensorTriggers = 0;

  // Check for a HIGH to LOW transition (magnet passing the sensor)
  if (sensorState == LOW && lastSensorState == HIGH) 
  {
    lastSensorState = sensorState;
    sensorTriggers++;
    if( sensorTriggers >3)
          sensorTriggers = 3;
  } 
  else if (sensorState == HIGH && lastSensorState == LOW) 
  {
    lastSensorState = sensorState;
  }
  else
  {
    if (millis() - lastSensorTriggerTime < 100) 
    {
        sensorTriggers--;
        if( sensorTriggers <=0)
          sensorTriggers = 0;
    }
  }

  if (sensorTriggers == 3) 
  {
    stoppedState = false;  // Pedal is moving
  }
  else 
  {
    stoppedState = true;  // Pedal is stopped
  }
}
