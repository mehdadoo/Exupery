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

void ThrottleSystem::setThrottleToZero()
{
  potValue1 = 0;
  potValue2 = 0;

  throttle1_percentage = 0;
  throttle2_percentage = 0;
}

void ThrottleSystem::chooseEngine()
{
  if (activeEngine == ENGINE_1 && speedSensor.getSpeed() > UP_SHIFT_SPEED)
    activeEngine = ENGINE_2;
  else if (activeEngine == ENGINE_2 && speedSensor.getSpeed() < DOWN_SHIFT_SPEED)
    activeEngine = ENGINE_1;
}

void ThrottleSystem::easeEnginePowerTowardsTarget()
{
  if (potValue1 < targetPotValue1)
    potValue1 ++;
  else if (potValue1 > targetPotValue1)
    potValue1 --;
  potValue1 = constrain(potValue1, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_1_MAX_VALUE); // Constrain the values (just in case)

  if (potValue2 < targetPotValue2)
    potValue2 +=2;
  else if (potValue2 > targetPotValue2)
    potValue2 --;
  potValue2 = constrain(potValue2, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_2_MAX_VALUE); // Constrain the values (just in case)
}

int ThrottleSystem::updateThrottleValue()
{
  //int joystick_knob = dashboard.joystick_knob;
  int joystick_throttle = dashboard.joystick_throttle;
  //map(joystick_knob, 0, JOYSTICK_THROTTLE_MAX_VALUE, 0, 100);

  if (joystick_throttle < JOYSTICK_THROTTLE_REST_MAX) 
    throttle_value = 0;
  else
    throttle_value = joystick_throttle;
  
  return throttle_value;
}

void ThrottleSystem::calculateTargetPotValues()
{
  if( activeEngine == ENGINE_1)
  {
    targetPotValue1 = map(throttle_value, JOYSTICK_THROTTLE_REST_MAX, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_1_MAX_VALUE);// Map joystick_throttle throttle zone to potValue2 from 30 to 63, more details in the constant definition line
    targetPotValue2 = std::max(targetPotValue2 - 1, 0);
  }
  else
  {
    targetPotValue1 = std::max(targetPotValue1 - 1, 0);
    targetPotValue2 = map(throttle_value, JOYSTICK_THROTTLE_REST_MAX, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_2_MAX_VALUE);// Map joystick_throttle throttle zone to potValue2 from 30 to 50, more details in the constant definition line
  }
}

void ThrottleSystem::limitMaxSpeed()
{
  if( speedSensor.getSpeed() > LIMIT_AUTHORISED_SPEED )
  {
    int currentPotValue2 = targetPotValue2;
    float speedRange = MAX_AUTHORISED_SPEED - LIMIT_AUTHORISED_SPEED; // Calculate the range between LIMIT_AUTHORISED_SPEED and MAX_AUTHORISED_SPEED
    float speedPercentage = (speedSensor.getSpeed() - LIMIT_AUTHORISED_SPEED) / speedRange;// Calculate the percentage of speed within this range
    speedPercentage = constrain(speedPercentage, 0, 1);// Ensure the percentage doesn't exceed 1 or go below 0
    potValue2 = currentPotValue2 - ((currentPotValue2 - POTENTIOMETER_MIN_VALUE) * speedPercentage); // Calculate the new potValue2 by reducing it from its current value to POTENTIOMETER_MIN_VALUE
    potValue2 = max(potValue2, POTENTIOMETER_MIN_VALUE);// Ensure potValue2 doesn't go below POTENTIOMETER_MIN_VALUE
  }
}

void ThrottleSystem::updatePotentiometerValues()
{
  potentiometer1.set(potValue1);
  potentiometer2.set(potValue2);

  throttle1_percentage = map(potValue1, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_1_MAX_VALUE, 0, 100);
  throttle2_percentage = map(potValue2, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_2_MAX_VALUE, 0, 100);

  throttle1_percentage = constrain(throttle1_percentage, 0, 100);
  throttle2_percentage = constrain(throttle2_percentage, 0, 100);
}

bool ThrottleSystem::enoughTimeHasPassed()
{
  unsigned long currentTime = millis();

  if (currentTime - lastThrottleUpdateTime > THROTTLE_UPDATE_EASE_SPEED)
  {
    lastThrottleUpdateTime = currentTime;
    return true;
  }

  return false;
}





// Method to update the potentiometer values
void ThrottleSystem::update()
{
  if( !dashboard.initialized || !initialized)
    return;

  if( /*pedalSensor.isStopped() ||*/ dashboard.hasBraked())
  {
    setThrottleToZero();
  }
  else
  {
    if ( enoughTimeHasPassed() )
    {
      if ( updateThrottleValue() == 0) 
      {
        setThrottleToZero();
      }
      else
      {
        chooseEngine();
        calculateTargetPotValues();
        easeEnginePowerTowardsTarget();
      }
    }
    limitMaxSpeed();
  }

  updatePotentiometerValues();
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
