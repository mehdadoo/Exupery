#include <Arduino.h>
#include "Buzzer.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "PortExpander.h"

// Initialize the MCP23S17

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
  static int beeps = 0;

  if (is_beeping_3)
  {
    if( beeps < 6 )
    {
      if ( millis() - beepStart > BUZZER_BEEP_DURATION ) 
      {
          beepStart = millis();
          beeps ++;
          beep_state = !beep_state;
          portExpander.digitalWriteMCP23S17(BUZZER_PIN, beep_state);
      } 
    }
    else 
    {
        is_beeping_3 = false;
        beeps = 0;
        portExpander.digitalWriteMCP23S17(BUZZER_PIN, LOW);
    }
  } 

  if (is_beeping_2)
  {
    if( beeps < 4 )
    {
      if ( millis() - beepStart > BUZZER_BEEP_DURATION ) 
      {
          beepStart = millis();
          beeps ++;
          beep_state = !beep_state;
          portExpander.digitalWriteMCP23S17(BUZZER_PIN, beep_state);
      } 
    }
    else 
    {
        is_beeping_2 = false;
        beeps = 0;
        portExpander.digitalWriteMCP23S17(BUZZER_PIN, LOW);
    }
  } 
}

void Buzzer::beep() 
{
  if (is_beeping_3)
    return;

  if (is_beeping_2)
    return;

  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
      return;

  portExpander.digitalWriteMCP23S17(BUZZER_PIN, HIGH);

  beepStart = millis();
  is_beeping = true;
  is_beeping_3 = false;
  is_beeping_2 = false;
}

void Buzzer::beep3() 
{
    PortExpander& portExpander = PortExpander::getInstance();

    if( !portExpander.initialized )
        return;

    beep_state = HIGH;
    portExpander.digitalWriteMCP23S17(BUZZER_PIN, beep_state);

    beepStart = millis();
    is_beeping_3 = true;
}

void Buzzer::beep2() 
{
    PortExpander& portExpander = PortExpander::getInstance();

    if( !portExpander.initialized )
        return;

    beep_state = HIGH;
    portExpander.digitalWriteMCP23S17(BUZZER_PIN, beep_state);

    beepStart = millis();
    is_beeping_2 = true;
}

void Buzzer::toggle() 
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
      return;

  if( is_beeping )
    portExpander.digitalWriteMCP23S17(BUZZER_PIN, LOW);
  else
    portExpander.digitalWriteMCP23S17(BUZZER_PIN, HIGH);

  is_beeping = !is_beeping;
}
void Buzzer::off() 
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
      return;

  portExpander.digitalWriteMCP23S17(BUZZER_PIN, LOW);
  is_beeping_3 = false;
  is_beeping_2 = false;
  is_beeping = false;
}
