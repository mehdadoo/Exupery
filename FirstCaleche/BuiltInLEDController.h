#ifndef BUILTIN_LED_CONTROLLER_H
#define BUILTIN_LED_CONTROLLER_H

#include <Arduino.h> // Include Arduino core library

class BuiltInLEDController 
{
  private:
    bool isBlinking = false;       // Flag to indicate if the LED is blinking
    unsigned long startTime = 0;   // Timer to track the blink duration

  public:
    // Constructor to initialize the LED controller
    BuiltInLEDController();

    // Initialize the LED pin
    void setup();

    // Start a blink
    void blink();

    void on();
    void off();

    // Update the LED state (to be called in loop)
    void update();
};

#endif