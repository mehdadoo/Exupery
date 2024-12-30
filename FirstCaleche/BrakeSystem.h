#ifndef BRAKESYSTEM_H
#define BRAKESYSTEM_H


#include <ESP32Servo.h>
#include "SpeedSensor.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "RotaryEncoder.h"


class BrakeSystem 
{
  public:
    BrakeSystem(SpeedSensor& sensorInstance); // Constructor with SpeedSensor parameter

    void setup();
    void update();

    int brakeLeverPosition;
    int servoPosition;

  private:
    // Private properties
    Servo servo;
    
    int lowerLeverThreshold;
    int upperLeverThreshold;
    RotaryEncoder rotary;     // the rotatory encoder in the brake lever, it is a ky040
    bool brakeSystemUpToDate;
    SpeedSensor& speedSensor; // an instance of the speed sensor passed in to the constructor

    // Private methods
    void setupServo();
    void updateBrakeLights();
    void updateServo();
    static void onBrakeLeverPositionChange(); // Static callback for rotary encoder
};

#endif



