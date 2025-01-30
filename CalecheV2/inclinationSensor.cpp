#include "InclinationSensor.h"
#include <Wire.h>
#include <math.h>
#include "ConstantDefinitions.h"

// Constructor
InclinationSensor::InclinationSensor() 
  : IMU(Wire, 0x68),  // Initialize the MPU9250 sensor with I2C and address 0x68
    lastUpdateTime(0),  // Initialize last update time
    initialized(false),  // Set the sensor as uninitialized
    inclination_angle(0.0)  // Initialize the inclination angle
{
}

void InclinationSensor::shutdown() 
{
  initialized = false;

}

// Method to initialize the sensor
void InclinationSensor::start() 
{
    if (initialized) 
        return;


    unsigned long module_connection_time_Start = millis();  // Record the time when the connection attempt starts

    int status = IMU.begin();  // Start the MPU9250 sensor

    if (status < 0) {
        WiFiPrinter::print("MPU initialization unsuccessful!");
    } else {
        unsigned long initialization_time = millis() - module_connection_time_Start;
        
        // Print the initialization time
        WiFiPrinter::print("MPU initialized in " + String(initialization_time) + " ms");

        initialized = true;
    }
}

// Method to update the sensor data and calculate inclination angle
void InclinationSensor::update() 
{
    if (!initialized) 
        return;

    unsigned long currentTime = millis();

    // Check if the required time interval has passed
    if (currentTime - lastUpdateTime < MPU_UPDATE_INTERVAL)
        return;  // Exit if the interval hasn't passed

    // Update the last update time
    lastUpdateTime = currentTime;

    // Read sensor data
    IMU.readSensor();

    float accelX = IMU.getAccelX_mss();  // Read X-axis acceleration
    float accelZ = IMU.getAccelZ_mss();  // Read Z-axis acceleration

    // Calculate the angle in degrees
    inclination_angle = atan2(accelX, accelZ) * 180.0 / PI;
}

// Method to get the current inclination angle
float InclinationSensor::getInclinationAngle() 
{
    return inclination_angle;
}
