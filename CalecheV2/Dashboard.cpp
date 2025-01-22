#include "Dashboard.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "PortExpander.h"

// Initialize the Dashboard
void Dashboard::start() 
{
  pinMode(VOLTMETER_SPEED, OUTPUT);
  pinMode(VOLTMETER_CHARGING, OUTPUT);
  pinMode(VOLTMETER_BATTERY, OUTPUT);
  

  unsigned long module_connection_time_Start = millis(); // Record the time when the connection attempt starts

  while (millis() - module_connection_time_Start < MODULE_CONNECTION_TIMEOUT)
  {
    if (ads1115.begin(0x4A)) 
    {
        initialized = true;
        ads1115.setGain(GAIN_ONE);

        // Calculate how long it took to initialize in ms
        unsigned long initialization_time = millis() - module_connection_time_Start;

        // Print the initialization time
        WiFiPrinter::print("ADS1115 joystick initialized in " + String( initialization_time ) + "ms");

        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }

  // If initialization failed after the timeout, call the error handling method
  if (!initialized) 
     WiFiPrinter::print( "Could not inilialize ADS1115 for joystick" );
}

// Deinitialize the Dashboard
void Dashboard::shutdown() 
{
  if( initialized)
  {
    setVoltmeterPWM(VOLTMETER_SPEED,    0,  VOLTMETER_SPEED_CHANNEL   );
    setVoltmeterPWM(VOLTMETER_CHARGING, 0,  VOLTMETER_CHARGING_CHANNEL);
    setVoltmeterPWM(VOLTMETER_BATTERY,  0,  VOLTMETER_BATTERY_CHANNEL );
  }

  initialized = false;
}

// Update the Dashboard (main logic)
void Dashboard::update() 
{
  if( !initialized)
    return;

  updateButtons();
  updateJoysticks();
  updateVoltmeters();
}

// Update button states
void Dashboard::updateButtons()
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
    return;

  static unsigned long lastLOWTime[4] = {0, 0, 0, 0};
  static const uint8_t mosfetPins[] = {MOSFET_1_PIN, MOSFET_2_PIN, MOSFET_3_PIN, MOSFET_4_PIN, MOSFET_5_PIN};
  static unsigned long provisionalButtonState[4] = {HIGH, HIGH, HIGH, HIGH};

  
    

  for (uint8_t i = 0; i < 4; i++)
  {
    uint8_t currentButtonState = portExpander.digitalRead( i );  // Read current state of button

    // If the button state has changed, print the new state
    if (currentButtonState != provisionalButtonState[i]) 
    {
      if (currentButtonState == HIGH) 
      {
        buttonState[i] = HIGH;
        portExpander.digitalWrite(mosfetPins[i], LOW);
        provisionalButtonState[i] = HIGH;
      }
      else 
      {
        lastLOWTime[i] = millis();
        provisionalButtonState[i] = LOW;
      }

    }

    if (currentButtonState == LOW && provisionalButtonState[i] == LOW &&  buttonState[i] == HIGH) 
    {
      if( millis() - lastLOWTime[i] > debounceDelay )
      {
        buttonState[i] = LOW;
        portExpander.digitalWrite(mosfetPins[i], HIGH);
      }
    }
  }


  lastDebounceTime = millis();
  if (millis() - lastDebounceTime > debounceDelay) 
  {
  }



}

// Update joystick states
void Dashboard::updateJoysticks()
{
  unsigned long currentTime = millis();  // Get the current time

  if (currentTime - joystickLastUpdateTime >= MPU_UPDATE_INTERVAL) 
  {  // Check if enough time has passed
    joystick_steering = readJoystick(3);
    joystick_throttle = constrain(readJoystick(0), 0, JOYSTICK_THROTTLE_MAX_VALUE);
    joystick_knob =     constrain(readJoystick(1), 0, JOYSTICK_THROTTLE_MAX_VALUE);
    
    joystickLastUpdateTime = currentTime;  // Update the last update time
  }
}

// Update voltmeter readings
void Dashboard::updateVoltmeters()
{
  unsigned long currentTime = millis();  // Get the current time

  // Check if enough time has passed
  if (currentTime - voltMetersLastUpdateTime >= VOLTMETER_UPDATE_INTERVAL) 
  {  
    
    //map the voltage percentage
    int pwmVoltagePercentageValue = map(batteryPercentage, 0, 100, 0, 255);// Map percentage to PWM range (0-255)

    setVoltmeterPWM(VOLTMETER_BATTERY, pwmVoltagePercentageValue,  VOLTMETER_CHARGING_CHANNEL);
    setVoltmeterPWM(VOLTMETER_SPEED,    joystick_steering,  VOLTMETER_SPEED_CHANNEL);
    setVoltmeterPWM(VOLTMETER_CHARGING,  joystick_knob,  VOLTMETER_BATTERY_CHANNEL);

    voltMetersLastUpdateTime = currentTime;  // Update the last update time
  }
}

// Set PWM for a voltmeter
void Dashboard::setVoltmeterPWM(int pin, int pwmValue, int channel, int freq, int resolution)
{
    // Ensure the PWM value is within the valid range
  if (pwmValue < 0) pwmValue = 0;
  if (pwmValue > 255) pwmValue = 255; // Maximum PWM value for 8-bit resolution

  // Attach the pin to a channel with frequency and resolution
  //ledcAttach(pin, freq, resolution);
  ledcAttachChannel(pin, freq, resolution, channel);

  // Write the PWM value directly to the pin
  ledcWrite(pin, pwmValue);
}

// Function to read joystick value from ADS1115 A2 pin and map to PWM
int Dashboard::readJoystick(int adcPin)
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

