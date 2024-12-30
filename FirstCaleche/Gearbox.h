#ifndef GEARBOX_H
#define GEARBOX_H

#include <ESP32Servo.h>
#include "SpeedSensor.h"

class Gearbox 
{
  public:
    // Constructor
    Gearbox(SpeedSensor& sensorInstance); // Constructor with SpeedSensor parameter

    // Public methods
    void setup();
    void update();

  private:
    // Private properties
    Servo servoGearbox;      // Servo object for the gearbox
    SpeedSensor& speedSensor; // an instance of the speed sensor passed in to the constructor

    // Private methods
    void updateServo();
};

#endif