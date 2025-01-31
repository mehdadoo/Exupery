#ifndef LCD_DISPLAY_H
#define LCD_DISPLAY_H

#define BACKGROUND BLACK
#define MARK_COLOR WHITE
#define NEEDLE_COLOR RED
#define CENTER_COLOR WHITE
#define GAUGE_MIN 0
#define GAUGE_MAX 100


#include <Arduino_GFX_Library.h>

class LCDDisplay 
{
  public:
    // Constructor
    LCDDisplay();

    // Public methods
    void update();
    void start();
    void shutdown();

    void updateGauge(int value);

  private:
    bool initialized = false;

    Arduino_DataBus *bus;
    Arduino_GFX *gfx;

    int current_value = 0;

    // Private methods
    void drawGauge();
    void drawNeedle(int value, uint16_t color);
};

#endif