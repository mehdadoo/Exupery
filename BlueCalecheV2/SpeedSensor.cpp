#include "SpeedSensor.h"
#include "WiFiPrinter.h"
#include <Arduino.h>

// Initialize the static instance
SpeedSensor* SpeedSensor::instance = nullptr;

// Constructor
SpeedSensor::SpeedSensor()
{
    // Set the static instance to this object
    rpm = 0;
    speed = 0.0;
    lastSensorTriggerTime = 0;
    pendingTriggerTime = 0;
    lastInterruptTriggerTime = 0;
    triggerPending = false;
    instance = this;
}


// Update the speed sensor
void SpeedSensor::update() 
{
    if (!ENABLE_SPEED_SENSOR)
        return;

    bool hasPendingTrigger = false;
    unsigned long triggerTime = 0;

    noInterrupts();
    if (triggerPending)
    {
        triggerTime = pendingTriggerTime;
        triggerPending = false;
        hasPendingTrigger = true;
    }
    interrupts();

    if (hasPendingTrigger)
    {
        if (lastSensorTriggerTime == 0)
        {
            // The first pulse establishes a baseline; speed needs two pulses.
            lastSensorTriggerTime = triggerTime;
        }
        else
        {
            calculateRPM(triggerTime);
        }
    }

    // If enough time has passed since the last trigger, consider the car stopped.
    unsigned long currentTime = millis();
    if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_3KMH) 
    {
        rpm = 0;
        speed = 0.0;
        lastSensorTriggerTime = 0;
    }
}



// Static interrupt handler
void IRAM_ATTR SpeedSensor::onTriggerSpeedSensor()
{
    if (!instance)
        return;

    unsigned long currentTime = millis();
    unsigned long previousInterruptTime = instance->lastInterruptTriggerTime;

    // Reject contact bounce and pulses above the configured maximum speed.
    if (previousInterruptTime != 0 &&
        currentTime - previousInterruptTime < SENSOR_INTERVAL_50KMH)
        return;

    instance->lastInterruptTriggerTime = currentTime;
    instance->pendingTriggerTime = currentTime;
    instance->triggerPending = true;
}

void SpeedSensor::calculateRPM(unsigned long triggerTime)
{
    unsigned long timeSinceLastTrigger = triggerTime - lastSensorTriggerTime;

    if (timeSinceLastTrigger == 0)
        return;

    // Calculate RPM (Revolutions Per Minute)
    rpm = (60 * 1000) / timeSinceLastTrigger;

    // Calculate speed in km/h using the wheel circumference
    float wheelCircumference = WHEEL_DIAMETER * INCHES_TO_METERS * 3.14159; // Circumference in meters
    speed = (rpm * wheelCircumference * 60) / 1000; // Speed in km/h

    // Record the time of this sensor trigger
    lastSensorTriggerTime = triggerTime;
}




bool SpeedSensor::isCarStopped() 
{
    // If enough time has passed since the last trigger, consider the car stopped
    unsigned long currentTime = millis();
    if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_3KMH) 
    {
        rpm = 0;
        speed = 0.0;
        lastSensorTriggerTime = 0;
        return true;
    }
    return false;
}


/////////////////////////////////////////////////////////////////////
////////////////////          Setup         /////////////////////////
/////////////////////////////////////////////////////////////////////
void SpeedSensor::setup() 
{
  if (!ENABLE_SPEED_SENSOR)
  {
    WiFiPrinter::print("SpeedSensor disabled");
    return;
  }

  //setup sensor pin
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), onTriggerSpeedSensor, FALLING);

  WiFiPrinter::print("SpeedSensor setup complete!");
}

