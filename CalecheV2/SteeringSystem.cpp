#include "SteeringSystem.h"

#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

// Constructor
SteeringSystem::SteeringSystem(Dashboard& dashboardInstance) 
  : dashboard(dashboardInstance), servoValue(0) // Initialize members
{
}

// Method to shutdown the throttle system
void SteeringSystem::shutdown()
{
  if( initialized)
  {
    servo.detach();
  }

  initialized = false;
}

// Method to update the potentiometer values
void SteeringSystem::update() 
{
	if( !initialized || !dashboard.initialized)
		return;
	
	int joystickMidpoint = (JOYSTICK_STEERING_MIN_VALUE + JOYSTICK_STEERING_MAX_VALUE) / 2; // Middle point of the joystick
	int servoMidpoint = (STERING_SERVO_MIN_VALUE + STERING_SERVO_MAX_VALUE) / 2;           // Middle point of the servo range

	if (abs(dashboard.joystick_steering - joystickMidpoint) <= JOYSTICK_STEERING_REST_GAP) 
	{
	// Within the rest gap: set servo to the midpoint
	servoValue = servoMidpoint;
	} 
	else if (dashboard.joystick_steering < joystickMidpoint - JOYSTICK_STEERING_REST_GAP) 
	{
	// Left portion of the joystick: map to the left servo range
	servoValue = map(dashboard.joystick_steering, 
							 JOYSTICK_STEERING_MIN_VALUE, 
							 joystickMidpoint - JOYSTICK_STEERING_REST_GAP, 
							 STERING_SERVO_MIN_VALUE, 
							 servoMidpoint);
	} 
	else 
	{
	// Right portion of the joystick: map to the right servo range
	servoValue = map(dashboard.joystick_steering, 
							 joystickMidpoint + JOYSTICK_STEERING_REST_GAP, 
							 JOYSTICK_STEERING_MAX_VALUE, 
							 servoMidpoint, 
							 STERING_SERVO_MAX_VALUE);
	}

	// Ensure the calculated servo value is within the allowed range
	servoValue = constrain(servoValue, STERING_SERVO_MIN_VALUE, STERING_SERVO_MAX_VALUE);

	// Write the value to the servo
	servo.write(servoValue);
}

// Method to start the throttle system
void SteeringSystem::start() 
{
	servo.attach(SERVO_STEERING);  // Reattach the servo if it was detached
	delay(50);
	initialized = true;
}
