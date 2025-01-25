#ifndef BRAKESYSTEM_H
#define BRAKESYSTEM_H

#include <ESP32Servo.h>
#include "SpeedSensor.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"


class BrakeSystem 
{
  public:
    BrakeSystem(SpeedSensor& sensorInstance); // Constructor with SpeedSensor parameter

    void start();
    void update();
    void shutdown();

    int brakeLeverPosition;

    int servoPosition1 = 0;
    int servoPosition2 = 0;

  private:
    // Private properties
    Servo servoBrake1; 
	  Servo servoBrake2; 
    
    SpeedSensor& speedSensor; // an instance of the speed sensor passed in to the constructor

    bool initialized = false;
    // Private methods
    void updateBrakeLights();
    void updateServo();
};

#endif



