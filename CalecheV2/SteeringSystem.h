#ifndef STEERING_SYSTEM_H
#define STEERING_SYSTEM_H

#include <ESP32Servo.h>
#include "Dashboard.h"

class SteeringSystem
{
  public:
    // Public methods
    SteeringSystem(Dashboard& dashboardInstance, SpeedSensor& speedSensorInstance);  // Constructor
    void shutdown();   // Method to shutdown the throttle system
    void update();     // Method to update the potentiometer values
    void start();      // Method to start the throttle system
    int servoValue;       
    int steering_percentage;

  private:
    // Private members

        // Value for the first potentiometer
    Servo servo; 
    bool initialized = false;
    Dashboard& dashboard;    // Reference to the Dashboard instance
     SpeedSensor& speedSensor;
};

#endif
