#ifndef STEERING_SYSTEM_H
#define STEERING_SYSTEM_H

#include <ESP32Servo.h>
#include "Dashboard.h"

class SteeringSystem
{
  public:
    // Public methods
    SteeringSystem(Dashboard& dashboardInstance);  // Constructor
    void shutdown();   // Method to shutdown the throttle system
    void update();     // Method to update the potentiometer values
    void start();      // Method to start the throttle system

  private:
    // Private members

    int servoValue;           // Value for the first potentiometer
    Servo servo; 
    bool initialized = false;
    Dashboard& dashboard;    // Reference to the Dashboard instance
};

#endif
