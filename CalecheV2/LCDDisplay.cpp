#include "LCDDisplay.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

// Constructor using an initializer list
LCDDisplay::LCDDisplay()
    : bus(new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED)),
      gfx(new Arduino_GC9A01(bus, TFT_RST, 0, true))
{
}

void LCDDisplay::update() 
{
 if( !initialized)
    return;

  for (int i = GAUGE_MIN; i <= GAUGE_MAX; i += 1) 
  {
    updateGauge(i);
    delay(50);
  }

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
        drawGauge();

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



void LCDDisplay::drawGauge() 
{
  int centerX = gfx->width() / 2;
  int centerY = gfx->height() / 2;
  int radius = min(centerX, centerY) - 10;

  for (int i = 0; i <= 100; i += 10) 
  {
    float angle = map(i, GAUGE_MIN, GAUGE_MAX, -135, 135) * DEG_TO_RAD;
    int x0 = centerX + cos(angle) * (radius - 10);
    int y0 = centerY + sin(angle) * (radius - 10);
    int x1 = centerX + cos(angle) * radius;
    int y1 = centerY + sin(angle) * radius;
    gfx->drawLine(x0, y0, x1, y1, MARK_COLOR);
  }

  gfx->fillCircle(centerX, centerY, 5, CENTER_COLOR);
}

void LCDDisplay::updateGauge(int value) 
{
  if (!initialized) 
    return;

  if (value < GAUGE_MIN) value = GAUGE_MIN;
  if (value > GAUGE_MAX) value = GAUGE_MAX;

  int centerX = gfx->width() / 2;
  int centerY = gfx->height() / 2;
  int radius = min(centerX, centerY) - 20;

  // Erase old needle
  drawNeedle(current_value, BACKGROUND);

  // Draw new needle
  drawNeedle(value, NEEDLE_COLOR);

  current_value = value;
}

void LCDDisplay::drawNeedle(int value, uint16_t color) 
{
  int centerX = gfx->width() / 2;
  int centerY = gfx->height() / 2;
  int radius = min(centerX, centerY) - 20;
  float angle = map(value, GAUGE_MIN, GAUGE_MAX, -135, 135) * DEG_TO_RAD;
  int x = centerX + cos(angle) * radius;
  int y = centerY + sin(angle) * radius;
  gfx->drawLine(centerX, centerY, x, y, color);
}