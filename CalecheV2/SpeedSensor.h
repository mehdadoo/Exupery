#ifndef SPEED_SENSOR_H
#define SPEED_SENSOR_H


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
    float getSpeed() const { return speed; }

  private:
    int rpm;       // Revolutions per minute
    float speed;     // Speed in km/h
    unsigned long lastSensorTriggerTime; // Last time the sensor detected the magnet
    bool initialized = false;

    // Private methods
    void calculateRPM();
};

#endif