#include "InclinationSensor.h"
#include <Wire.h>
#include <math.h>
#include "ConstantDefinitions.h"
#include "WiFiPrinter.h"

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
  
  IMU._stateMachine = 0;
}

// Method to initialize the sensor
void InclinationSensor::start() 
{
    if (initialized) 
        return;

    if( IMU._stateMachine <= 0)
    {
      module_connection_time_Start = millis();
      IMU.begin();  // Start the MPU9250 sensor
    }
}

// Method to update the sensor data and calculate inclination angle
void InclinationSensor::update() 
{
    IMU.update();

    static int previousStateMachine = -1; // Store the previous state

    if (IMU._stateMachine != previousStateMachine) 
    {
        //WiFiPrinter::print("InclinationSensor initializing: " + String(IMU._stateMachine) + " of 11");
        unsigned long initialization_time = millis() - module_connection_time_Start;
        
        if(IMU._stateMachine == 11 )
        {
            initialized = true;
            WiFiPrinter::print("InclinationSensor initialized in " + String( initialization_time ) + "ms");
        }
        else if(IMU._stateMachine < 0 )
        {
          // Print the initialization time
          WiFiPrinter::print("InclinationSensor NOT initialized after " + String( initialization_time ) + "ms");
        }

        previousStateMachine = IMU._stateMachine; // Update previous state
    }

    

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
