#include "VoltageSensor.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"


// Initialize the Dashboard
void VoltageSensor::start() 
{
  unsigned long module_connection_time_Start = millis(); // Record the time when the connection attempt starts
  while (millis() - module_connection_time_Start < MODULE_CONNECTION_TIMEOUT)
  {
    if (ads1115.begin()) 
    {
        initialized = true;
        ads1115.setGain(GAIN_ONE);

        unsigned long initialization_time = millis() - module_connection_time_Start;
        
        // Print the initialization time
        WiFiPrinter::print("ADS1115 for voltageSensor initialized in " + String( initialization_time ) + "ms");

        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }


  // If initialization failed after the timeout, call the error handling method
  if (!initialized) 
     WiFiPrinter::print( "Could not inilialize ADS1115 for voltageSensor" );
}

// Deinitialize the VoltageSensor
void VoltageSensor::shutdown() 
{
  if( initialized)
  {
    
  }

  initialized = false;
}

// Update the VoltageSensor (main logic)
void VoltageSensor::update() 
{
  if( !initialized)
    return;

  unsigned long currentTime = millis();

  // Check if the required time interval has passed
  if (currentTime - lastUpdateTime < CURRENTSENSOR_UPDATE_INTERVAL)
  {
    return; // Exit the method if the interval hasn't passed
  }

  // Update the last update time
  lastUpdateTime = currentTime;

  voltage = readBatteryVoltage();
  current = readCurrentSensor(2);

  //Calculate voltage percentage
  int constrained_voltage = constrain(voltage , MIN_BATTERY_VOLTAGE, MAX_BATTERY_VOLTAGE);// Constrain voltage to the valid range
  batteryPercentage = map(constrained_voltage * 100, MIN_BATTERY_VOLTAGE * 100, MAX_BATTERY_VOLTAGE * 100, 0, 100);// Map voltage to percentage (0-100)
}

float VoltageSensor::readBatteryVoltage() 
{
    int16_t adcReading = readCurrentSensor(0); // Get ADC reading


    // Calculate and print battery voltage

    float resolutionPerBit = 0.000125; // 0.125 mV/bit for ADS1115 with GAIN_ONE
    float slope = 0.01604;
    float dividerFactor = 0.0642; // Calculate the divider factor
    return adcReading * slope / dividerFactor;// * (resolutionPerBit / dividerFactor); // Calculate battery voltage
}

  
// Function to read joystick value from ADS1115 A2 pin and map to PWM
int VoltageSensor::readCurrentSensor( int adcPin ) 
{
  // Read ADC value (16-bit signed integer, -32768 to 32767)
  int16_t rawValue = ads1115.readADC_SingleEnded( adcPin );

  // Convert raw ADC value to PWM range (0-255)
  // ADS1115 range: 0-32767 maps to 0-3.3V
  int pwmValue = map(rawValue, 0, 32767, 0, PWM_RESOLUTION);

  // Ensure PWM value is within the valid range
  pwmValue = constrain(pwmValue, 0, PWM_RESOLUTION);

  return pwmValue;
}
