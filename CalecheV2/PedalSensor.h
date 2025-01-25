#ifndef PEDAL_SENSOR_H
#define PEDAL_SENSOR_H

#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

class PedalSensor 
{
  public:
    // Public methods
    void update();
    bool isStopped() { return stoppedState; }

  private:
    bool stoppedState; 
    unsigned long lastSensorTriggerTime = 0; // Last time the sensor detected the magnet

    // Private methods
    void calculateRPM();
};

#endif