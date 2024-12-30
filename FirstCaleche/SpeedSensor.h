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
    void setup();
    void update();
    bool isCarStopped(); // Check if RPM is 0
    int getRPM() const { return rpm; }
    int getSpeed() const { return speed; }

  private:
    int rpm;       // Revolutions per minute
    int speed;     // Speed in km/h
    unsigned long lastSensorTriggerTime; // Last time the sensor detected the magnet

    // Private methods
    void handleSpeedSensor();
    void calculateRPM();

    // Static for the interrupt
    static void onTriggerSpeedSensor();
    static SpeedSensor* instance; // Singleton-like instance for static callback
};

#endif