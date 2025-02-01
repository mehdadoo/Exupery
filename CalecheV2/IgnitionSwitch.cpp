#include "IgnitionSwitch.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include <Arduino.h>
#include "WiFiPrinter.h"
#include "Buzzer.h"

void IgnitionSwitch::setup() 
{
  unsigned long module_connection_time_Start = millis();

  // Initialize pins
	pinMode(POWER_SWITCH_PIN, INPUT_PULLUP);
	pinMode(MOSFET_48V_PIN, OUTPUT);
  neopixelWrite(RGB_BUILTIN,  14,  14,  0);
  shutdown();

  unsigned long initialization_time = millis() - module_connection_time_Start;
  WiFiPrinter::print("Setup Complete in " + String( initialization_time ) + "ms");
}

void IgnitionSwitch::start() 
{
  unsigned long module_connection_time_Start = millis();

  isKeyOn = true;
  neopixelWrite(RGB_BUILTIN, 0, 0, 24); // Blue LED
  digitalWrite(MOSFET_48V_PIN, LOW);
  delay( IGNITION_MOSFET_STARTUP_DELAY );
  onTurnedOn();
  neopixelWrite(RGB_BUILTIN, 0, 64, 0); // Green LED

  unsigned long initialization_time = millis() - module_connection_time_Start;
  WiFiPrinter::print("Car start in " + String( initialization_time ) + "ms");
  Buzzer::getInstance().beep();
}
void IgnitionSwitch::shutdown() 
{
  Buzzer::getInstance().beep();
  delay(BUZZER_BEEP_DURATION);
  Buzzer::getInstance().off();

  isKeyOn = false;
  onTurnedOff();
  digitalWrite(MOSFET_48V_PIN, HIGH);
  neopixelWrite(RGB_BUILTIN, 64, 0, 0); // Red LED
  WiFiPrinter::print("Car shutdown");
  
}
void IgnitionSwitch::update() 
{
  int currentState = digitalRead(POWER_SWITCH_PIN);

  if (currentState == LOW && !isKeyOn) 
  { 
    // Key turned ON → Activate immediately
    start();
  } 
  else if (currentState == HIGH && isKeyOn) 
  { 
      // Key turned OFF → Start 300ms timer
      if (!waitingForOff) 
      {
          keyOffTime = millis();
          waitingForOff = true;
      }

      // If key remains OFF for 300ms, confirm shutdown
      if (waitingForOff && (millis() - keyOffTime >= 300)) 
      {
        shutdown();
        waitingForOff = false;
      }
  } 
  else if (currentState == LOW) 
  { 
      waitingForOff = false; // If key turns back ON before 300ms → Cancel shutdown
  }
}

void IgnitionSwitch::setOnTurnedOnListener(std::function<void()> callback) {
    onTurnedOn = callback;
}

void IgnitionSwitch::setOnTurnedOffListener(std::function<void()> callback) {
    onTurnedOff = callback;
}
