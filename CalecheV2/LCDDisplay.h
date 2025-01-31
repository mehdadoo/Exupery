#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#define BACKGROUND BLACK
#define MARK_COLOR WHITE
#define NEEDLE_COLOR RED
#define CENTER_COLOR WHITE
#define GAUGE_MIN 0
#define GAUGE_MAX 100

#include "Arduino_DataBus.h"
#include "Arduino_GFX.h"

class LCDDisplay 
{
  public:
    // Constructor
    LCDDisplay();

    // Public methods
    void update();
    void start();
    void shutdown();

    void updateDisplay(
        bool button1, bool button2, bool button3, bool button4, 
        int speedSensor, int pedalSensor,
        int joystick_throttle, int joystick_knob, int joystick_steering,
        float voltage, float current, float inclinationAngle);

    void updateGauge(int value);

  private:
    bool initialized = false;
    unsigned long lastDisplayUpdate = 0;

    Arduino_DataBus *bus;
    Arduino_GFX *gfx;

    // Private methods
    void drawButtonIndicator(int x, int y, bool state);
    void drawSlider(int x, int y, int value, const char* label);
    void drawCircularGauge(int x, int y, float value, float minVal, float maxVal, const char* label);
    void drawBarIndicator(int x, int y, float value, float minVal, float maxVal, const char* label);
    void drawInclinationArrow(int x, int y, float angle);

};

#endif