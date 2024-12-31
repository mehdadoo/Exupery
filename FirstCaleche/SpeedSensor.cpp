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
    instance = this;
}


// Update the gearbox
void SpeedSensor::update() 
{
  // If enough time has passed since the last trigger, consider the car stopped
    unsigned long currentTime = millis();
    if (currentTime - lastSensorTriggerTime > SENSOR_INTERVAL_3KMH) 
    {
        rpm = 0;
        speed = 0.0;
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
// Actual handler for the speed sensor
void SpeedSensor::handleSpeedSensor() 
{
    static int lastSensorState = HIGH; // Track the last state of the sensor

    // Read the current state of the sensor
    int sensorState = digitalRead(SPEED_SENSOR_PIN);

    // Check for a HIGH to LOW transition (magnet passing the sensor)
    if (sensorState == LOW && lastSensorState == HIGH) 
    {
        unsigned long currentTime = millis();
        
        // Ensure there is enough time since last calculation
        if (currentTime - lastSensorTriggerTime >= SENSOR_INTERVAL_50KMH) 
        {
            // Calculate RPM and Speed
            calculateRPM();
        }

        digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
    }
    else if (sensorState == HIGH && lastSensorState == LOW)
    {
        digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
    }

    // Update the last sensor state
    lastSensorState = sensorState;
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


/////////////////////////////////////////////////////////////////////
////////////////////          Setup         /////////////////////////
/////////////////////////////////////////////////////////////////////
void SpeedSensor::setup() 
{
  //setup sensor pin
  pinMode(SPEED_SENSOR_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SPEED_SENSOR_PIN), onTriggerSpeedSensor, FALLING);

  //setup led
  pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW); // Start with the LED off

  WiFiPrinter::print("SpeedSensor setup complete!");
}

