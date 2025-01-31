#ifndef PEDAL_SENSOR_H
#define PEDAL_SENSOR_H


class PedalSensor 
{
  public:
    // Public methods
    void update();
    void start();
    void shutdown();
    bool isStopped() { return stoppedState; }
    int sensorTriggers = 0;

  private:
    bool stoppedState; 
    unsigned long lastSensorTriggerTime = 0; // Last time the sensor detected the magnet
    bool initialized = false;

    // Private methods
    void calculateRPM();
};

#endif