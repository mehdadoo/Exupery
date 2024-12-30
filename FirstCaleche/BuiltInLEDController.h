#ifndef BUILTIN_LED_CONTROLLER_H
#define BUILTIN_LED_CONTROLLER_H

#include <Arduino.h> // Include Arduino core library

class BuiltInLEDController 
{
  private:

  public:
    // Constructor to initialize the LED controller
    BuiltInLEDController();

    // Initialize the LED pin
    void setup();
    void on();
    void off();
    void update();
};

#endif