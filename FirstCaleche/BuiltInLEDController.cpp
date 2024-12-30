#include "BuiltInLEDController.h"
#include "ConstantDefinitions.h"

BuiltInLEDController::BuiltInLEDController() 
{}

// Start a blink
void BuiltInLEDController::blink()
{
	isBlinking = true;       // Set the blinking flag
	startTime = millis();    // Record the start time
	digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
}

void BuiltInLEDController::on()
{
	digitalWrite(LED_BUILTIN, HIGH); // Turn the LED on
}
void BuiltInLEDController::off()
{
	digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
}

// Update the LED state
void BuiltInLEDController::update() 
{
	if (isBlinking)
	{
		if (millis() - startTime >= SPEED_SENSOR_LED_BLINK_DURATION) 
		{
		  digitalWrite(LED_BUILTIN, LOW); // Turn the LED off
		  isBlinking = false;     // Reset the blinking flag
		}
  }
}

// Initialize the LED pin
void BuiltInLEDController::setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW); // Start with the LED off
}