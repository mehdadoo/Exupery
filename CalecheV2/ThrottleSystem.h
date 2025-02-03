#ifndef THROTTLE_SYSTEM_H
#define THROTTLE_SYSTEM_H

#include <DigiPotX9Cxxx.h>
#include "Dashboard.h"
#include "PedalSensor.h"
#include "SpeedSensor.h"

class ThrottleSystem
{
  public:
    // Public methods
    ThrottleSystem(Dashboard& dashboardInstance, PedalSensor& sensorInstance, SpeedSensor& speedSensorInstance);  // Constructor
    void shutdown();   // Method to shutdown the throttle system
    void update();     // Method to update the potentiometer values
    void start();      // Method to start the throttle system

    int throttle1_percentage;           // Value for the first potentiometer
    int throttle2_percentage;           // Value for the second potentiometer

  private:
    // Private members
    DigiPot potentiometer1;  // First potentiometer
    DigiPot potentiometer2;  // Second potentiometer

    int potValue1;           // Value for the first potentiometer
    int targetPotValue1 = 0;
    int potValue2;           // Value for the second potentiometer
    int targetPotValue2 = 0;

    unsigned long lastThrottleUpdateTime;
    bool initialized;

    Dashboard& dashboard;    // Reference to the Dashboard instance
    PedalSensor& pedalSensor; // an instance of the speed sensor passed in to the constructor
    SpeedSensor& speedSensor;
};

#endif
