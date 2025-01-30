#include "VoltageSensor.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"


// Initialize the Dashboard
void VoltageSensor::start() 
{
  WiFiPrinter::print("VoltageSensor initialized");
  pinMode(VOLTAGE_DIRECT_PIN, INPUT);
  initialized = true;
}

// Deinitialize the VoltageSensor
void VoltageSensor::shutdown() 
{
  initialized = false;
}

// Update the VoltageSensor (main logic)
void VoltageSensor::update() 
{
  if( !initialized)
    return;
  
  unsigned long currentTime = millis();

  // Check if the required time interval has passed
  if (currentTime - lastUpdateTime < VOLTAGE_SENSOR_UPDATE_INTERVAL)
  {
    return; // Exit the method if the interval hasn't passed
  }

  // Update the last update time
  lastUpdateTime = currentTime;

  voltage = readBatteryVoltage();

  //Calculate voltage percentage
  int constrained_voltage = constrain(voltage , MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE);// Constrain voltage to the valid range
  batteryPercentage = map(constrained_voltage * 100, MIN_BATTERY_VOLTAGE * 100, MAX_BATTERY_VOLTAGE * 100, 0, 100);// Map voltage to percentage (0-100)
}

float VoltageSensor::readBatteryVoltage() 
{
  int adcValue = analogRead(VOLTAGE_DIRECT_PIN);
  float voltage = (adcValue / 4095.0) * 3.3;
  return voltage / VOLTAGE_SENSOR_DEVIDER_FACTOR;
}
