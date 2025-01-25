#ifndef DASHBOARD_H
#define DASHBOARD_H

#include <Adafruit_ADS1X15.h>

class Dashboard {
public:
    // Public member variables
    int joystick_throttle = 0;
    int joystick_steering = 0;
    int joystick_knob = 0;
    bool buttonState[4]; // Button states
    bool toggleState[4];
    bool initialized = false;
    int batteryPercentage = 100;

    // Public methods
    void start();
    void shutdown();
    void update();

private:
    // Private member variables
    Adafruit_ADS1115 ads1115;
    unsigned long voltMetersLastUpdateTime = 0;  // Variable to store the last update time
    unsigned long joystickLastUpdateTime = 0;  // Variable to store the last update time
    unsigned long lastDebounceTime = 0;
    const long debounceDelay = 5;  // milliseconds

    // Private methods
    int readJoystick(int adcPin);
    void updateButtons();
    void updateJoysticks();
    void updateVoltmeters();
    void setVoltmeterPWM(int pin, int pwmValue, int channel, int freq = 5000, int resolution = 8);
};

#endif // DASHBOARD_H
