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

    void setup();
    void update();
    void toggleHandBrake();
    void setSimulatedBrakeLeverPosition(int position);
    void usePhysicalBrakeLever();

    int brakeLeverPosition;
    int brakeLeverRawValue;
    int frontServoPosition;
    int backServoPosition;
    bool handBrakeEnabled;
    bool simulatedBrakeLeverEnabled;

  private:
    // Private properties
    Servo frontServo;
    Servo backServo;
    
    int lowerLeverThreshold;
    int upperLeverThreshold;
    long filteredPotReading;
    unsigned long lastPotSampleTime;
    bool potFilterInitialized;
    bool lastHandBrakeButtonReading;
    bool stableHandBrakeButtonState;
    unsigned long handBrakeDebounceStartTime;
    bool brakeSystemUpToDate;
    SpeedSensor& speedSensor; // an instance of the speed sensor passed in to the constructor

    // Private methods
    int readBrakeLeverPosition();
    void updateHandBrakeButton();
    void setupServo();
    void updateBrakeLights();
    void updateServo();
};

#endif



