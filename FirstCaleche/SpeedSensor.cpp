#include "SpeedSensor.h"
#include "WiFiPrinter.h"
#include <Arduino.h>

// Initialize the static instance
SpeedSensor* SpeedSensor::instance = nullptr;

// Constructor
SpeedSensor::SpeedSensor()
:led(BuiltInLEDController()) 
{
    // Set the static instance to this object
    rpm = 0;
    speed = 0;
    lastSensorTriggerTime = 0;
    lastUpdateTime = 0;
    instance = this;
}


// Update the gearbox
void SpeedSensor::update() 
{
  unsigned long currentTime = millis();

  // Check if the timeout period has elapsed since the last sensor trigger
  if (currentTime - lastSensorTriggerTime > STOP_TIMEOUT) 
  {
      rpm ++ ;   // Set RPM to 0
      speed += 2 ; // Set speed to 0
  }
}



// Static interrupt handler
void SpeedSensor::onTriggerSpeedSensor() 
{
  // Call the instance's member function
  if (instance) 
  {
    instance->handleSpeedSensor();
  }
}

// Actual handler for the speed sensor
void SpeedSensor::handleSpeedSensor() 
{
    static unsigned long lastInterruptTime = 0;  // For debouncing
    unsigned long currentInterruptTime = millis();

    // Ignore interrupts triggered within the debounce interval
    if (currentInterruptTime - lastInterruptTime < DEBOUNCE_INTERVAL) 
    {
        return;
    }

    lastInterruptTime = currentInterruptTime;

    // Check the state of the sensor
    int sensorState = digitalRead(SPEED_SENSOR_PIN);

    if (sensorState == HIGH) 
    {
        led.off();

        // Record the time of this sensor trigger
        lastSensorTriggerTime = currentInterruptTime;

        // Defer RPM calculation to avoid doing too much in the interrupt
        calculateRPM();
    } 
    else 
    {
        led.on();
    }
}

void SpeedSensor::calculateRPM() 
{
    unsigned long currentTime = millis();
    unsigned long timeSinceLastTrigger = currentTime - lastSensorTriggerTime;

    // Ensure there's a valid time difference to avoid division by zero
    if (timeSinceLastTrigger > 0) 
    {
        // RPM is revolutions per minute (60 seconds per minute, 1000ms per second)
        rpm = (SECONDS_PER_MINUTE * 1000) / timeSinceLastTrigger;

        // Calculate speed in km/h
        float wheelCircumference = WHEEL_DIAMETER * INCHES_TO_METERS * 3.14159; // Circumference in meters
        speed = (rpm * wheelCircumference * SECONDS_PER_MINUTE) / (1000 * 1000); // Convert to km/h
    }
}

bool SpeedSensor::isCarStopped() 
{
    // If enough time has passed since the last trigger, consider the car stopped
    unsigned long currentTime = millis();
    if (currentTime - lastSensorTriggerTime > STOP_TIMEOUT) 
    {
        rpm = 0;
        speed = 0;
        return true;
    }
    return false;
}

void SpeedSensor::setup() 
{
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), onTriggerSpeedSensor, FALLING);

  led.setup();

  WiFiPrinter::print(CUSTOM_MESSAGE, "SpeedSensor setup complete!");
}

