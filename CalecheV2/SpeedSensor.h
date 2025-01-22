#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

class SpeedSensor 
{
  public:
    // Constructor
    SpeedSensor();

    // Public methods
    void update();
    bool isCarStopped(); // Check if RPM is 0
    int getRPM() const { return rpm; }
    float getSpeed() const { return speed; }

  private:
    int rpm;       // Revolutions per minute
    float speed;     // Speed in km/h
    unsigned long lastSensorTriggerTime; // Last time the sensor detected the magnet

    // Private methods
    void calculateRPM();
};

#endif