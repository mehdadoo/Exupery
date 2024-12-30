// NightLights.h
#ifndef NIGHTLIGHTS_H
#define NIGHTLIGHTS_H

#include "PinDefinitions.h"

class NightLights 
{
  public:
    // Constructor
    NightLights();

    // Public methods
    void setup();
    void update();

  private:
    int previousState; // Variable to store the previous state of the lights
};

#endif
