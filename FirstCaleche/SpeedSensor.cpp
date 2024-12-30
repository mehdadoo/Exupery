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
  led.update();
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
  int sensorState = digitalRead(SPEED_SENSOR_PIN);

  // Check if the pin is HIGH or LOW
  if (sensorState == HIGH) 
  {
    led.off();

    // Calculate RPM based on the time difference
    lastSensorTriggerTime = millis();
    calculateRPM();

    WiFiPrinter::print(RPM_CONSTANT, rpm);
    WiFiPrinter::print(SPEED_CONSTANT, speed); 
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

    // Update RPM only if there's a valid time difference
    if (timeSinceLastTrigger > 0) 
    {
        rpm = (SECONDS_PER_MINUTE * 1000) / timeSinceLastTrigger;

        // Calculate speed in km/h
        float wheelCircumference = WHEEL_DIAMETER * INCHES_TO_METERS * 3.14159; // Circumference in meters
        speed = (rpm * wheelCircumference * SECONDS_PER_MINUTE) / 1000; // Convert to km/h
    }

    lastUpdateTime = currentTime;
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
  Serial.println("SpeedSensor setup complete!");
}

