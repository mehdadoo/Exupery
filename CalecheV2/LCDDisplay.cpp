#include "LCDDisplay.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "databus/Arduino_ESP32SPI.h"
#include "display/Arduino_GC9A01.h"

// Constructor using an initializer list
LCDDisplay::LCDDisplay()
    : bus(new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, MISO_PIN)),
      gfx(new Arduino_GC9A01(bus, TFT_RST, 0, true))
{
}

void LCDDisplay::update() 
{
  /*if( !initialized)
    return;

  unsigned long currentMillis = millis();

  if (currentMillis - lastDisplayUpdate >= (1000 / DISPLAY_FPS)) 
  {
      lastDisplayUpdate = currentMillis;

      // Your display update code here
      updateGauge(v);
      v += direction;
  }*/
}

void LCDDisplay::shutdown() 
{
  if( initialized)
  {
    gfx->fillScreen(BACKGROUND);
  }

  initialized = false;
}

void LCDDisplay::start() 
{
  unsigned long module_connection_time_Start = millis(); // Record the time when the connection attempt starts

  while (millis() - module_connection_time_Start < MODULE_CONNECTION_TIMEOUT)
  {
    if ( gfx->begin() ) 
    {
        gfx->fillScreen(BACKGROUND);
        //drawGauge();

        unsigned long initialization_time = millis() - module_connection_time_Start; // Calculate how long it took to initialize in ms
        WiFiPrinter::print("LCDDisplay initialized in " + String( initialization_time ) + "ms");// Print the initialization time
        initialized = true;
        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }

  // If initialization failed after the timeout, call the error handling method
  if (!initialized) 
     WiFiPrinter::print( "Could NOT inilialize LCDDisplay" );
}



void LCDDisplay::updateDisplay(
    bool button1, bool button2, bool button3, bool button4, 
    int speedSensor, int pedalSensor,
    int joystick_throttle, int joystick_knob, int joystick_steering,
    float voltage, float current, float inclinationAngle)
{
    if (!initialized) 
        return;

    unsigned long currentMillis = millis();
    if (currentMillis - lastDisplayUpdate < (1000 / DISPLAY_FPS))
        return;

    lastDisplayUpdate = currentMillis;
    //gfx->fillScreen(BACKGROUND);

    int centerX = gfx->width() / 2;
    int centerY = gfx->height() / 2;
    
    // Draw buttons as indicators
    drawButtonIndicator(centerX - 50, 20, button1);
    drawButtonIndicator(centerX - 25, 20, button2);
    drawButtonIndicator(centerX + 25, 20, button3);
    drawButtonIndicator(centerX + 50, 20, button4);

    // Draw Sliders
    drawSlider(20, 50, speedSensor, "Speed");
    drawSlider(20, 70, pedalSensor, "Pedal");
    drawSlider(20, 90, joystick_throttle, "Throttle");
    drawSlider(20, 110, joystick_knob, "Knob");
    drawSlider(20, 130, joystick_steering, "Steering");

    // Draw Voltage Gauge
    //drawCircularGauge(centerX, centerY, voltage, 0, 60, "VOLT");
    drawBarIndicator(centerX + 50, centerY - 80, voltage, 30, 60, "Volts");

    // Draw Current as Bar Indicator
    drawBarIndicator(centerX + 50, centerY - 20, current, 0, 20, "Amp");

    // Draw Inclination Arrow
    drawInclinationArrow(centerX, centerY + 60, inclinationAngle);
}

// ======================== DRAW HELPER FUNCTIONS ========================

// Draws small LED indicators for buttons
void LCDDisplay::drawButtonIndicator(int x, int y, bool state)
{
    uint16_t color = state ? GREEN : RED;
    gfx->fillCircle(x, y, 5, color);
}

// Draws a horizontal slider
void LCDDisplay::drawSlider(int x, int y, int value, const char* label)
{
    gfx->fillRect(x, y, 80, 8, BLACK);
    gfx->drawRect(x, y, 80, 8, WHITE);
    gfx->fillRect(x+1, y+1, map(value, 0, 100, 0, 80)-2, 8-2, BLUE);
    gfx->setCursor(x + 85, y);
    gfx->print(label);
}

// Draws a circular voltage gauge
void LCDDisplay::drawCircularGauge(int x, int y, float value, float minVal, float maxVal, const char* label)
{
    float angle = map(value, minVal, maxVal, -135, 135) * DEG_TO_RAD;
    int radius = 40;

    // Draw tick marks
    for (int i = -135; i <= 135; i += 30) 
    {
        int tx = x + cos(i * DEG_TO_RAD) * radius;
        int ty = y + sin(i * DEG_TO_RAD) * radius;
        gfx->drawPixel(tx, ty, WHITE);
    }

    // Draw needle
    int nx = x + cos(angle) * (radius - 5);
    int ny = y + sin(angle) * (radius - 5);
    gfx->drawLine(x, y, nx, ny, RED);

    gfx->setCursor(x - 10, y + 50);
    gfx->print(label);
}

// Draws a vertical bar for current
void LCDDisplay::drawBarIndicator(int x, int y, float value, float minVal, float maxVal, const char* label)
{
    int height = 40;
    int barHeight = map(value, minVal, maxVal, 0, height);
    gfx->fillRect(x, y, 8, height, BLACK);
    gfx->drawRect(x, y, 8, height, WHITE);
    gfx->fillRect(x, y + height - barHeight, 8, barHeight, GREEN);
    gfx->setCursor(x + 12, y + height / 2);
    gfx->print(label);
}

// Draws an arrow for inclination angle
void LCDDisplay::drawInclinationArrow(int x, int y, float angle)
{
    float rad = angle * DEG_TO_RAD;
    int length = 20;
    int arrowX = x + cos(rad) * length;
    int arrowY = y + sin(rad) * length;
    
    gfx->fillCircle(x, y, 40,  BLACK);
    gfx->drawLine(x, y, arrowX, arrowY, YELLOW);
    gfx->fillCircle(arrowX, arrowY, 3, YELLOW);
}