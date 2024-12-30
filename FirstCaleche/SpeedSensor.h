#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H

#include "BuiltInLEDController.h"
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
    // Private properties
    BuiltInLEDController led;       // LED controller for indicating activity
    
    int rpm;       // Revolutions per minute
    int speed;     // Speed in km/h
    unsigned long lastSensorTriggerTime; // Last time the sensor detected the magnet
    unsigned long lastUpdateTime;        // Last time RPM was updated


    // Private methods
    void handleSpeedSensor();
    void calculateRPM();

    // Static for the interrupt
    static void onTriggerSpeedSensor();
    static SpeedSensor* instance; // Singleton-like instance for static callback
};

#endif