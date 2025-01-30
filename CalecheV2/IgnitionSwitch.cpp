#include "IgnitionSwitch.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include <Arduino.h>
#include "WiFiPrinter.h"

IgnitionSwitch::IgnitionSwitch() 
{
    // Constructor
}

void IgnitionSwitch::setup() 
{
    // Initialize pins
	pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOSFET_48V_PIN, OUTPUT);

  
  neopixelWrite(RGB_BUILTIN,  0,  0,  64);
  WiFiPrinter::print("Setup Complete");
}

void IgnitionSwitch::update() 
{
	// Read the current state of the car key switch
	int currentState = digitalRead(POWER_SWITCH_PIN);

  if( currentState != isKeyOn)
  {
    shouldUpdate = true;
  }

	// Check if the state has changed
	if ( shouldUpdate ) 
	{
		if (currentState == LOW) 
    {
      neopixelWrite(RGB_BUILTIN,0,0,24); // blue led
      digitalWrite(MOSFET_48V_PIN, LOW); 

      delay( 200 );

		  onTurnedOn();

      neopixelWrite(RGB_BUILTIN,0,64,0); // Green led
      WiFiPrinter::print("Car Turned On");
    }
		else 
    {
		  onTurnedOff();
      digitalWrite( MOSFET_48V_PIN, HIGH); 
      delay(200);
      neopixelWrite(RGB_BUILTIN,  64,  0,  0); // full red led
      WiFiPrinter::print("Car Turned Off");

      
    }
    isKeyOn = currentState;
    shouldUpdate = false;
	}
  // Update the previous state
}

void IgnitionSwitch::setOnTurnedOnListener(std::function<void()> callback) {
    onTurnedOn = callback;
}

void IgnitionSwitch::setOnTurnedOffListener(std::function<void()> callback) {
    onTurnedOff = callback;
}
