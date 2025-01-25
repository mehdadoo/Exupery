#include "BrakeSystem.h"
#include "ConstantDefinitions.h"
#include "PortExpander.h"
#include "WiFiPrinter.h"
#include <algorithm> // For std::clamp


// Constructor
BrakeSystem::BrakeSystem(SpeedSensor& sensorInstance): speedSensor(sensorInstance)
{
}


void BrakeSystem::update() 
{
  if( initialized  )
  {
    updateBrakeLights();
    updateServo();
  }
}

//Update using a ease method, so that at lower positions of the lever, the servo reacts 3 times more compared to the highest lever position
void BrakeSystem::updateServo()
{
  PortExpander& portExpander = PortExpander::getInstance();

  bool skipUpdate = false;

  int joystick_throttle = brakeLeverPosition;

  if (joystick_throttle > JOYSTICK_THROTTLE_SERVO_BRAKE_MIN)
  {
    if( servoPosition1 == BRAKE_SERVO_MIN_VALUE)
      skipUpdate = true;

    servoPosition1 = BRAKE_SERVO_MIN_VALUE; // Set servoPosition1 to max if above 100
    servoPosition2 = BRAKE_SERVO_MAX_VALUE; // Set servoPosition1 to 0 if above 100, this servo is in reverse!
  }
  else
  {
    // Map joystick_throttle from 100 to 200 to brake from 0 to 270
    servoPosition1 = map(joystick_throttle, JOYSTICK_THROTTLE_SERVO_BRAKE_MAX, JOYSTICK_THROTTLE_SERVO_BRAKE_MIN, BRAKE_SERVO_MAX_VALUE, BRAKE_SERVO_MIN_VALUE);
    servoPosition2 = map(joystick_throttle, JOYSTICK_THROTTLE_SERVO_BRAKE_MAX, JOYSTICK_THROTTLE_SERVO_BRAKE_MIN, BRAKE_SERVO_MIN_VALUE, BRAKE_SERVO_MAX_VALUE);
  }

  if (joystick_throttle > JOYSTICK_THROTTLE_REST_MIN)
      portExpander.digitalWrite(MOSFET_BRAKE_PIN, LOW);
  else
      portExpander.digitalWrite(MOSFET_BRAKE_PIN, HIGH);

  if( skipUpdate )
    return;


  servoBrake1.write( servoPosition1 );
  servoBrake2.write( servoPosition2 );
}

void BrakeSystem::updateBrakeLights()
{
   PortExpander& portExpander = PortExpander::getInstance();

  static unsigned long positionInRangeStartTime = 0; // Time when lever entered the range
  static bool lightState = false; // State of the light
  static unsigned long lastBlinkTime = 0; // Last time the light blinked

  unsigned long currentTime = millis();

  // Check if the brake lever is below the lower threshold to turn off the light
  if (brakeLeverPosition > JOYSTICK_THROTTLE_REST_MIN) 
  {
    portExpander.digitalWrite(MOSFET_BRAKE_LIGHT_PIN, LOW); // Brake light off
    lightState = false; // Ensure state is consistent
  } 
  else
  {
    if ( speedSensor.isStopped() ) 
    {
      // Full brightness: Keep the light on
      portExpander.digitalWrite(MOSFET_BRAKE_LIGHT_PIN, HIGH);
      lightState = true; // Ensure state is consistent
    } 
    else 
    {
      // Calculate the blink interval based on the lever position
      int range = JOYSTICK_THROTTLE_SERVO_BRAKE_MAX - JOYSTICK_THROTTLE_REST_MIN;
      int positionInRange = brakeLeverPosition - JOYSTICK_THROTTLE_REST_MIN;
      int multiplier = range == 0 ? 1 : map(positionInRange, 0, range, BRAKE_BLINK_MULTIPLIER, 1); // Inverse multiplier from 10 to 1
      int blinkInterval = multiplier * BRAKE_BLINK_RATE;

      // Blink the brake light with the calculated interval
      if (currentTime - lastBlinkTime >= blinkInterval) 
      {
          lightState = !lightState; // Toggle the light state
          portExpander.digitalWrite(MOSFET_BRAKE_LIGHT_PIN, lightState ? HIGH : LOW);
          lastBlinkTime = currentTime; // Reset the timer
      }
    }
  } 
}


void BrakeSystem::start() 
{
  servoBrake1.attach(SERVO_BRAKE_1); 
  servoBrake2.attach(SERVO_BRAKE_2);

  delay(50);

  initialized = true;
  
  WiFiPrinter::print("BrakeSystem setup complete!");
}

void BrakeSystem::shutdown() 
{
  if( initialized)
  {
    servoBrake1.detach();
    servoBrake2.detach();
  }

  initialized = false;
}
