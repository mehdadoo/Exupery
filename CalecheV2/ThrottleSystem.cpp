#include "ThrottleSystem.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

// Constructor to initialize the potentiometers and set initial values
// Constructor to initialize the potentiometers and set initial values
ThrottleSystem::ThrottleSystem(Dashboard& dashboardInstance, PedalSensor& sensorInstance)
  : potentiometer1(DigiPot_NC, DigiPot_UD, DigiPot_CS1),
    potentiometer2(DigiPot_NC, DigiPot_UD, DigiPot_CS2),
    dashboard(dashboardInstance),      // Initialize the dashboard reference
    pedalSensor(sensorInstance)        // Initialize the pedal sensor reference
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

  // Set potentiometer control pins to INPUT to prevent backfeeding
  digitalWrite(DigiPot_CS1, HIGH); // Pull CS high before disabling
  digitalWrite(DigiPot_CS2, HIGH);
  pinMode(DigiPot_CS1, INPUT);
  pinMode(DigiPot_CS2, INPUT);
  pinMode(DigiPot_NC, INPUT);
  pinMode(DigiPot_UD, INPUT);

  // Add a delay for stabilization
  delay(10);

  initialized = false;
}

// Method to update the potentiometer values
void ThrottleSystem::update()
{
  if( !dashboard.initialized || !initialized)
    return;


  if( pedalSensor.isStopped() || dashboard.hasBraked() )
  {
    potValue1 = 0;
    potValue2 = 0;

    potentiometer1_percentage = 0;
    potentiometer2_percentage = 0;
  }
  else
  {
    int joystick_knob = dashboard.joystick_knob;
    int joystick_throttle = dashboard.joystick_throttle;

    potValue1 = map(joystick_knob, 0, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);
    potValue2 = 0; // Initialize potValue2

    if (joystick_throttle < JOYSTICK_THROTTLE_REST_MAX) 
        potValue2 = 0; // Set potValue2 to 0 if below 100
    else
      potValue2 = map(joystick_throttle, JOYSTICK_THROTTLE_REST_MAX, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);// Map joystick_throttle from 100 to 200 to potValue2 from 0 to 75
  }

  // Apply the value to the potentiometers
  // Ensure potValue never exceeds 75
  potValue1 = min(potValue1, POTENTIOMETER_MAX_VALUE);
  potValue2 = min(potValue2, POTENTIOMETER_MAX_VALUE); 

  potentiometer1_percentage = map(potValue1, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE, 0, 100);
  potentiometer2_percentage = map(potValue2, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE, 0, 100);

  potentiometer1.set(potValue2);
  potentiometer2.set(potValue1);
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
