#include "SpeedSensor.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "Buzzer.h"
#include <Arduino.h>


// Global instance for ISR
volatile bool sensorTriggered = false;
// ISR function (called when sensor state changes)
void IRAM_ATTR sensorISR() 
{
    sensorTriggered = true;
}


// Constructor
SpeedSensor::SpeedSensor()
{
  // Set the static instance to this object
  rpm = 0;
  speed = 0.0;
  lastSensorTriggerTime = millis();
  lastSensorTriggerDuration = SENSOR_INTERVAL_50KMH;
}

void SpeedSensor::start() 
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized)
    return;

  initialized = true;
  attachInterrupt(digitalPinToInterrupt(MCP23S17_INT_PIN), sensorISR, FALLING); // Attach interrupt to the MCP23S17 INT pin, Adjust FALLING/RISING as needed
}

void SpeedSensor::shutdown() 
{
  initialized = false;
  detachInterrupt(digitalPinToInterrupt(MCP23S17_INT_PIN));
}

// Update the gearbox
void SpeedSensor::update() 
{
  if (!initialized) return;

  unsigned long currentTime = millis();
  
  if( currentTime - lastSensorTriggerTime > lastSensorTriggerDuration)
    calculateRPM();

  if (sensorTriggered) 
  {
    sensorTriggered = false;

    unsigned long currentTime = millis();
    if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_50KMH) 
    {
      calculateRPM();
      saveTriggerTime();
      Buzzer::getInstance().beep();
    }
  }
}

void SpeedSensor::calculateRPM() 
{
  unsigned long currentTime = millis();
  unsigned long timeSinceLastTrigger = currentTime - lastSensorTriggerTime;

  if (timeSinceLastTrigger > 0)
  {
    rpm = (60 * 1000) / timeSinceLastTrigger;

    float wheelCircumference = WHEEL_DIAMETER * INCHES_TO_METERS * 3.14159;
    speed = (rpm * wheelCircumference * 60) / 1000;
  }
}


void SpeedSensor::saveTriggerTime() 
{
  unsigned long currentTime = millis();
  // Record the time of this sensor trigger
  lastSensorTriggerDuration = currentTime - lastSensorTriggerTime;
  lastSensorTriggerTime = currentTime;
}


bool SpeedSensor::isStopped() 
{
  // If enough time has passed since the last trigger, consider the car stopped because we are cycling less than 3km/h
  unsigned long currentTime = millis();
  if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_3KMH) 
  {
    return true;
  }
  return false;
}

