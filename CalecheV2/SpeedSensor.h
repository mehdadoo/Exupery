#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H
#include <Arduino.h>

class SpeedSensor 
{
  public:
    // Constructor
    SpeedSensor();

    // Public methods
    void update();
    void start();
    void shutdown();
    bool isStopped(); // Check if RPM is 0
    int getRPM() const { return rpm; }
    float getSpeed() const { return averageSpeed; }

  private:
    int rpm;          // Revolutions per minute
    float speed;     // Speed in km/h
    float averageSpeed;
    unsigned long lastSensorTriggerTime; // Last time the sensor detected the magnet
    unsigned long lastSensorTriggerDuration; // Last time the sensor detected the magnet
    bool initialized = false;
    // Private methods
    void calculateRPM();
    void saveTriggerTime();
};

#endif