// NightLights.cpp
#include "NightLights.h"
#include "WiFiPrinter.h"
#include "ConstantDefinitions.h"
#include <Arduino.h> // Include the Arduino core library

// Constructor
NightLights::NightLights() : previousState(LOW) {}  // Initialize previousState to LOW

// Setup method
void NightLights::setup() 
{
    pinMode(LIGHT_NIGHT_PIN, OUTPUT);
    pinMode(LIGHT_NIGHT_SWITCH_PIN, INPUT_PULLUP);

    bool currentState = digitalRead(LIGHT_NIGHT_SWITCH_PIN);
    digitalWrite(LIGHT_NIGHT_PIN, !currentState);
}

// Update method
void NightLights::update() 
{
    int currentState = digitalRead(LIGHT_NIGHT_SWITCH_PIN); // Read current switch state

    // Only change light state if it has changed from the previous state
    if (currentState == LOW && previousState == HIGH) 
    {
        digitalWrite(LIGHT_NIGHT_PIN, HIGH);
        WiFiPrinter::print("Night lights ON");

    } 
    else if (currentState == HIGH && previousState == LOW) 
    {
        digitalWrite(LIGHT_NIGHT_PIN, LOW);
        WiFiPrinter::print("Night lights OFF");
    }

    // Update the previous state for the next comparison
    previousState = currentState;
}
