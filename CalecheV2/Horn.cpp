#include <Arduino.h>
#include "Horn.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "PortExpander.h"

// Initialize the MCP23S17

// Update function (implement as needed)
void Horn::update() 
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
      return;

  if (is_beeping && millis() - beepStart > HORN_DURATION) 
  {
      is_beeping = false;
      portExpander.digitalWriteMCP23S17(MOSFET_HORN_PIN, LOW);
  } 
}

void Horn::beep() 
{
    PortExpander& portExpander = PortExpander::getInstance();

    if( !portExpander.initialized )
        return;

    portExpander.digitalWriteMCP23S17(MOSFET_HORN_PIN, HIGH);

    beepStart = millis();
    is_beeping = true;
}
