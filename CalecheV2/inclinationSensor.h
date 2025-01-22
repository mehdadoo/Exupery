#ifndef INCLINATION_SENSOR_H
#define INCLINATION_SENSOR_H

#include "MPU9250.h"
#include "WiFiPrinter.h"


class InclinationSensor 
{
public:
    InclinationSensor();  // Constructor
    void start();  // Method to initialize the sensor
    void update();  // Method to update the sensor readings and calculations
    float getInclinationAngle();  // Method to get the current inclination angle
    
private:
    MPU9250 IMU;  // MPU9250 sensor object
    unsigned long lastUpdateTime;  // Last time the sensor was updated
    bool initialized;  // Flag to check if the sensor is initialized
    float inclination_angle;  // Current inclination angle
};

#endif // INCLINATION_SENSOR_H
