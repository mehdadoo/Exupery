#ifndef BRAKESYSTEM_H
#define BRAKESYSTEM_H

#include <ESP32Servo.h>
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "Dashboard.h"
#include "SpeedSensor.h"


class BrakeSystem 
{
  public:
    BrakeSystem(Dashboard& dashboardInstance, SpeedSensor& sensorInstance); // Constructor with SpeedSensor parameter

    void start();
    void update();
    void shutdown();

    int servoPosition1 = 0;
    int servoPosition2 = 0;

  private:
    // Private properties
    Servo servoBrake1; 
	  Servo servoBrake2; 
    
    Dashboard& dashboard;    // Reference to the Dashboard instance
    SpeedSensor& speedSensor; // an instance of the speed sensor passed in to the constructor

    bool initialized = false;
    // Private methods
    void updateBrakeLights();
    void updateServo();
};

#endif



