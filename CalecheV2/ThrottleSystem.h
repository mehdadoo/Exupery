#ifndef THROTTLE_SYSTEM_H
#define THROTTLE_SYSTEM_H

#include <DigiPotX9Cxxx.h>
#include "Dashboard.h"
#include "PedalSensor.h"

class ThrottleSystem
{
  public:
    // Public methods
    ThrottleSystem(Dashboard& dashboardInstance, PedalSensor& sensorInstance);  // Constructor
    void shutdown();   // Method to shutdown the throttle system
    void update();     // Method to update the potentiometer values
    void start();      // Method to start the throttle system

  private:
    // Private members
    DigiPot potentiometer1;  // First potentiometer
    DigiPot potentiometer2;  // Second potentiometer

    int potValue1;           // Value for the first potentiometer
    int potValue2;           // Value for the second potentiometer

    bool initialized;

    Dashboard& dashboard;    // Reference to the Dashboard instance
    PedalSensor& pedalSensor; // an instance of the speed sensor passed in to the constructor
};

#endif
