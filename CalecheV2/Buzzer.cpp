#include <Arduino.h>
#include "Buzzer.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "PortExpander.h"

// Initialize the MCP23S17
void Buzzer::start() 
{
    initialized = true;
}

// Deinitialize the MCP23S17
void Buzzer::shutdown()
{
  if( !initialized )
  {
    
  }

  initialized = false;
}

// Update function (implement as needed)
void Buzzer::update() 
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
      return;

  if (is_beeping && millis() - beepStart > BUZZER_BEEP_DURATION) 
  {
      is_beeping = false;
      portExpander.digitalWriteMCP23S17(BUZZER_PIN, LOW);
  } 
}

void Buzzer::beep() 
{
    PortExpander& portExpander = PortExpander::getInstance();

    if( !portExpander.initialized )
        return;

    portExpander.digitalWriteMCP23S17(BUZZER_PIN, HIGH);

    beepStart = millis();
    is_beeping = true;
}
