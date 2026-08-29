#include "NightLights.h"
#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include <Arduino.h>

NightLights::NightLights()
    : lightsOn(false),
      lastButtonReading(HIGH),
      stableButtonState(HIGH),
      debounceStartTime(0)
{
}

void NightLights::setup()
{
  pinMode(NIGHT_LIGHT_MOSFET_PIN, OUTPUT);
  digitalWrite(NIGHT_LIGHT_MOSFET_PIN, LOW);

  if (ENABLE_NIGHT_LIGHT_BUTTON)
  {
    pinMode(NIGHT_LIGHT_BUTTON_PIN, INPUT_PULLUP);
    lastButtonReading = digitalRead(NIGHT_LIGHT_BUTTON_PIN);
    stableButtonState = lastButtonReading;
  }
}

void NightLights::update()
{
  if (!ENABLE_NIGHT_LIGHT_BUTTON)
    return;

  bool buttonReading = digitalRead(NIGHT_LIGHT_BUTTON_PIN);
  unsigned long currentTime = millis();

  if (buttonReading != lastButtonReading)
  {
    lastButtonReading = buttonReading;
    debounceStartTime = currentTime;
  }

  if (currentTime - debounceStartTime >= NIGHT_LIGHT_DEBOUNCE_MS &&
      buttonReading != stableButtonState)
  {
    stableButtonState = buttonReading;

    if (stableButtonState == LOW)
      toggle();
  }
}

void NightLights::toggle()
{
  setLights(!lightsOn);
}

void NightLights::setLights(bool enabled)
{
  lightsOn = enabled;
  digitalWrite(NIGHT_LIGHT_MOSFET_PIN, lightsOn ? HIGH : LOW);
  WiFiPrinter::print(lightsOn ? "Night lights ON" : "Night lights OFF");
}
