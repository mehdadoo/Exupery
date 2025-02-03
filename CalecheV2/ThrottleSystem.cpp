#include "ThrottleSystem.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include <algorithm> // For std::fill

// Constructor to initialize the potentiometers and set initial values
// Constructor to initialize the potentiometers and set initial values
ThrottleSystem::ThrottleSystem(Dashboard& dashboardInstance, PedalSensor& sensorInstance, SpeedSensor& speedSensorInstance)
  : potentiometer1(DigiPot_NC, DigiPot_UD, DigiPot_CS1),
    potentiometer2(DigiPot_NC, DigiPot_UD, DigiPot_CS2),
    dashboard(dashboardInstance),  
    pedalSensor(sensorInstance),
    speedSensor(speedSensorInstance) 
{
}


// Method to shutdown the throttle system, disable potentiometers
void ThrottleSystem::shutdown()
{
  // Set potentiometer values to 0 or other shutdown behavior
  potValue1 = 0;
  potValue2 = 0;

  if( initialized )
  {
    potentiometer1.set(potValue1);  // Apply shutdown value to potentiometer 1
    potentiometer2.set(potValue2);  // Apply shutdown value to potentiometer 2
  }

  // Add a delay for stabilization
  delay(10);

  initialized = false;
}

// Method to update the potentiometer values
void ThrottleSystem::update()
{
  if( !dashboard.initialized || !initialized)
    return;

  if( pedalSensor.isStopped() || dashboard.hasBraked() || speedSensor.getSpeed() > MAX_AUTHORISED_SPEED )
  {
    potValue1 = 0;
    potValue2 = 0;

    throttle1_percentage = 0;
    throttle2_percentage = 0;
  }
  else
  {

    unsigned long currentTime = millis();

    if (currentTime - lastThrottleUpdateTime > THROTTLE_UPDATE_EASE_SPEED)
    {
      //int joystick_knob = dashboard.joystick_knob;
      int joystick_throttle = dashboard.joystick_throttle;

      //potValue1 = map(joystick_knob, 0, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);
      //potValue2 = 0; // Initialize potValue2

      if (joystick_throttle < JOYSTICK_THROTTLE_REST_MAX) 
      {
          potValue2 = 0; // Set potValue2 to 0 if below 100
      }
      else
      {
        targetPotValue2 = map(joystick_throttle, JOYSTICK_THROTTLE_REST_MAX, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);// Map joystick_throttle throttle zone to potValue2 from 30 to 63, more details in the constant definition line
    
        // Adjust potValue2 towards targetPotValue2 (same logic)
        if (potValue2 < targetPotValue2)
          potValue2 ++;
        else if (potValue2 > targetPotValue2)
          potValue2 --;

        // Constrain the values (just in case)
        potValue2 = constrain(potValue2, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);
      }

      lastThrottleUpdateTime = currentTime;
    }
  }

  potentiometer1.set(potValue2);
  potentiometer2.set(potValue1);

  throttle1_percentage = map(potValue1, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE, 0, 100);
  throttle2_percentage = map(potValue2, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE, 0, 100);

  throttle1_percentage = constrain(throttle1_percentage, 0, 100);
  throttle2_percentage = constrain(throttle2_percentage, 0, 100);
}

// Method to start the throttle system, enabling potentiometer control
void ThrottleSystem::start()
{
  // Reinitialize potentiometer pins
  pinMode(DigiPot_CS1, OUTPUT);
  pinMode(DigiPot_CS2, OUTPUT);

  // Code to start the system or enable throttling
  // You may want to set initial values or enable any specific features at start
  potValue1 = 0;
  potValue2 = 0;

  potentiometer1.set(potValue1);
  potentiometer2.set(potValue2);

  initialized = true;
}
