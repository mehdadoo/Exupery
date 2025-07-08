#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

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
        bool handbrake, 
        float speed, 
        float voltage, 
        int brakePercentage,
        int throttle1_perentage, int throttle2_perentage,
        float inclinationAngle,
        int potValue1,
        int potValue2
        );

        

    void updateGauge(int value);

  private:
    bool initialized = false;
    unsigned long lastDisplayUpdate = 0;

    Arduino_DataBus *bus;
    Arduino_GFX *gfx;

    // Private methods
    void clearTextBox(int x, int y, float value);
    void drawTextBox(int x, int y, float value, const char* text);
    void drawArcSlider(int value, bool isLeft, uint16_t fillColor, uint16_t emptyColor);
    void drawBrakeSlider(int value, uint16_t fillColor, uint16_t emptyColor);
    void drawButtonIndicator(int x, int y, bool state);
    void drawSlider(int x, int y, int value, const char* label);
    void drawInclinationArrow(int x, int y, float angle);

};

#endif