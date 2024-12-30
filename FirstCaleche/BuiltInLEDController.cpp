#include "BuiltInLEDController.h"
#include "ConstantDefinitions.h"

BuiltInLEDController::BuiltInLEDController() 
{}

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
}

// Initialize the LED pin
void BuiltInLEDController::setup()
{
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW); // Start with the LED off
}