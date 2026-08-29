#ifndef NIGHTLIGHTS_H
#define NIGHTLIGHTS_H

#include "PinDefinitions.h"

class NightLights
{
  public:
    NightLights();

    void setup();
    void update();
    void toggle();
    bool isOn() const { return lightsOn; }

  private:
    bool lightsOn;
    bool lastButtonReading;
    bool stableButtonState;
    unsigned long debounceStartTime;

    void setLights(bool enabled);
};

#endif
