#include "SpeedSensor.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include <Arduino.h>

// Constructor
SpeedSensor::SpeedSensor()
{
    // Set the static instance to this object
    rpm = 0;
    speed = 0.0;
    lastSensorTriggerTime = 0;
}


// Update the gearbox
void SpeedSensor::update() 
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
    return;

  unsigned long currentTime = millis();
  
  // If enough time has passed since the last trigger, consider the car stopped
  if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_3KMH) 
  {
      rpm = 0;
      speed = 0.0;
  }
  

  uint8_t sensorState = portExpander.digitalRead(SENSOR_WHEEL_SPEED_PIN);  // Read current state of sensor
  static int lastSensorState = LOW;

  // Check for a HIGH to LOW transition (magnet passing the sensor)
  if (sensorState == LOW && lastSensorState == HIGH) 
  {
    lastSensorState = sensorState;
    unsigned long currentTime = millis();
    
    // Ensure there is enough time since last calculation
    if (currentTime - lastSensorTriggerTime >= SENSOR_INTERVAL_50KMH) 
    {
        // Calculate RPM and Speed
        calculateRPM();
    }
  }
  else if (sensorState == HIGH && lastSensorState == LOW)
  {
    lastSensorState = sensorState;
  }
  
}

void SpeedSensor::calculateRPM() 
{
    unsigned long currentTime = millis();
    unsigned long timeSinceLastTrigger = currentTime - lastSensorTriggerTime;

    // Calculate RPM (Revolutions Per Minute)
    rpm = (60 * 1000) / timeSinceLastTrigger;

    // Calculate speed in km/h using the wheel circumference
    float wheelCircumference = WHEEL_DIAMETER * INCHES_TO_METERS * 3.14159; // Circumference in meters
    speed = (rpm * wheelCircumference * 60) / 1000; // Speed in km/h

    // Record the time of this sensor trigger
    lastSensorTriggerTime = currentTime;
}




bool SpeedSensor::isCarStopped() 
{
    // If enough time has passed since the last trigger, consider the car stopped
    unsigned long currentTime = millis();
    if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_3KMH) 
    {
        rpm = 0;
        speed = 0.0;
        return true;
    }
    return false;
}

