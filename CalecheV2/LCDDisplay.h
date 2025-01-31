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
    void start();
    void shutdown();

    void update(
        bool button1, bool button2, bool button3, bool button4, 
        float speed, bool pedalSensorisStopped,
        int joystick_throttle, int joystick_knob, int joystick_steering,
        float voltage, int steeringPercentage, int brakePercentage, int throttle1_perentage, int throttle2_perentage, float inclinationAngle);

    void updateGauge(int value);

  private:
    bool initialized = false;
    unsigned long lastDisplayUpdate = 0;

    Arduino_DataBus *bus;
    Arduino_GFX *gfx;

    // Private methods
    void drawTextBox(int x, int y, float value, const char* text);
    void drawButtonIndicator(int x, int y, bool state);
    void drawSlider(int x, int y, int value, const char* label);
    void drawInclinationArrow(int x, int y, float angle);

};

#endif