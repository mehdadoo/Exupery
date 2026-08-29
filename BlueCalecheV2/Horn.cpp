#include "Horn.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include <Arduino.h>

Horn::Horn()
    : beepStart(0),
      debounceStartTime(0),
      isBeeping(false),
      lastButtonReading(HIGH),
      stableButtonState(HIGH)
{
}

void Horn::setup()
{
  pinMode(HORN_MOSFET_PIN, OUTPUT);
  digitalWrite(HORN_MOSFET_PIN, LOW);

  if (ENABLE_HORN_BUTTON)
  {
    pinMode(HORN_BUTTON_PIN, INPUT_PULLUP);
    lastButtonReading = digitalRead(HORN_BUTTON_PIN);
    stableButtonState = lastButtonReading;
  }
}

void Horn::update()
{
  unsigned long currentTime = millis();

  if (isBeeping && currentTime - beepStart >= HORN_DURATION)
  {
    isBeeping = false;
    digitalWrite(HORN_MOSFET_PIN, LOW);
  }

  if (!ENABLE_HORN_BUTTON)
    return;

  bool buttonReading = digitalRead(HORN_BUTTON_PIN);

  if (buttonReading != lastButtonReading)
  {
    lastButtonReading = buttonReading;
    debounceStartTime = currentTime;
  }

  if (currentTime - debounceStartTime >= HORN_BUTTON_DEBOUNCE_MS &&
      buttonReading != stableButtonState)
  {
    stableButtonState = buttonReading;

    if (stableButtonState == LOW)
      beep();
  }
}

void Horn::beep()
{
  digitalWrite(HORN_MOSFET_PIN, HIGH);
  beepStart = millis();
  isBeeping = true;
}
