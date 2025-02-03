#include "SpeedSensor.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "Buzzer.h"
#include <Arduino.h>

int lastSensorState = LOW; // Global instance for ISR
volatile bool sensorTriggered = false; // ISR function (called when sensor state changes)
void IRAM_ATTR handleMCP23S17Interrupt() 
{
  if( lastSensorState == HIGH)
    sensorTriggered = true; // Flag for main loop
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
  attachInterrupt(digitalPinToInterrupt(MCP23S17_INT_PIN), handleMCP23S17Interrupt, FALLING); // Attach interrupt to the MCP23S17 INT pin, Adjust FALLING/RISING as needed
}

void SpeedSensor::shutdown() 
{
  initialized = false;
  detachInterrupt(digitalPinToInterrupt(MCP23S17_INT_PIN));
}

// Update the gearbox
void SpeedSensor::update() 
{
  if (!initialized)
    return;

  PortExpander& portExpander = PortExpander::getInstance();
  unsigned long currentTime = millis();
  lastSensorState = portExpander.digitalReadMCP23S17(SENSOR_WHEEL_SPEED_PIN);

  if( currentTime - lastSensorTriggerTime > lastSensorTriggerDuration)
    calculateRPM();
  
  if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_50KMH) 
  {
    if (sensorTriggered) 
    {
      //1. act upon
      calculateRPM();
      saveTriggerTime();
      //Buzzer::getInstance().beep();
      
      //2. clear flags for next read
      sensorTriggered = false;
      portExpander.clearInterrupt();
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

    if (speed <= 3.0)
        speed = 0.0;

    // Moving Average Filter
    static float speedBuffer[SPEED_SAMPLES] = {0};  
    static int speedIndex = 0;  
    static float speedSum = 0;  

    speedSum -= speedBuffer[speedIndex];  // Remove oldest value
    speedBuffer[speedIndex] = speed;      // Store new speed value
    speedSum += speed;                    // Add new value to sum

    speedIndex = (speedIndex + 1) % SPEED_SAMPLES;  // Circular buffer update

    averageSpeed = speedSum / SPEED_SAMPLES;  // Compute smoothed speed
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
  return (speed == 0.0);
}