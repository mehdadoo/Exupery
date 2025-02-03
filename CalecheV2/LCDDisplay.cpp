#include "LCDDisplay.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "databus/Arduino_ESP32SPI.h"
#include "display/Arduino_GC9A01.h"
#include "Buzzer.h"
#include "FreeMono8pt7b.h"
#include "FreeSansBold10pt7b.h"
#include "FreeSerifBoldItalic12pt7b.h"


// Constructor using an initializer list
LCDDisplay::LCDDisplay()
    : bus(new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, MISO_PIN)),
      gfx(new Arduino_GC9A01(bus, TFT_RST, 0, true))
{
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
        gfx->setRotation(2);
        gfx->fillScreen(BACKGROUND);

        unsigned long initialization_time = millis() - module_connection_time_Start; // Calculate how long it took to initialize in ms
        WiFiPrinter::print("LCDDisplay initialized in " + String( initialization_time ) + "ms");// Print the initialization time
        initialized = true;
        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }

  // If initialization failed after the timeout, call the error handling method
  if (!initialized) 
  {
     WiFiPrinter::print( "Could NOT inilialize LCDDisplay" );
     Buzzer::getInstance().beep3();
  }

}



void LCDDisplay::update(
        bool button1, bool button2, bool button3, bool button4, 
        float speed, bool speedSensorIsStopped, bool pedalSensorIsStopped,
        float voltage, 
        int steeringPercentage,
        int brakePercentage,
        int throttle1_perentage, int throttle2_perentage,
        float inclinationAngle)
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
    //drawButtonIndicator(centerX - 30, 50, button2);
    //drawButtonIndicator(centerX - 10, 50, button1);
    //drawButtonIndicator(centerX + 10, 50, button4);
    drawButtonIndicator(centerX, centerY + 50, button3);

    drawTextBox(80, centerY - 30, speed, "KM/H");
    drawTextBox(80, centerY +  0, voltage, "v");
    drawTextBox(80, centerY + 30, inclinationAngle, "°");

    //drawSlider(60, 110, steeringPercentage, "Steering");
    //drawButtonIndicator(centerX - 10, 170, pedalSensorIsStopped);
    //drawButtonIndicator(centerX + 10, 170, speedSensorIsStopped);

    drawBrakeSlider(  brakePercentage,              0xd867, 0x5823);
    drawArcSlider(    throttle1_perentage, true,    0x9648, 0x4b25);
    drawArcSlider(    throttle2_perentage, false,   0x7e1d, 0x2a8d);
}

// ======================== DRAW HELPER FUNCTIONS ========================

void LCDDisplay::drawTextBox(int x, int y, float value, const char* label) 
{
    // Define a fixed-size box for text clearing
    int textWidth = 80;  // Adjust width as needed
    int textHeight = 40; // Adjust height as needed

    // Erase previous value by drawing a black rectangle
    gfx->fillRect(x, y-20, textWidth, textHeight, BLACK);

    // Draw the new text
    gfx->setTextColor(WHITE);
    gfx->setFont(&FreeSansBold10pt7b);
    //gfx->setTextSize(2);
    gfx->setCursor(x + 5, y + 5); // Small padding inside the box
    gfx->print( String(value, 1));

    gfx->setTextColor( 0x9492 );
    //gfx->setTextSize(3);
    gfx->setFont(&FreeMono8pt7b);
    gfx->setCursor(x + 47, y + 5); // Small padding inside the box
    gfx->print( String(label) );
}


// Draws small LED indicators for buttons
void LCDDisplay::drawButtonIndicator(int x, int y, bool state)
{
    uint16_t color = state ? GREEN : RED;
    gfx->fillCircle(x, y, 5, color);
}

void LCDDisplay::drawArcSlider(int value, bool isLeft, uint16_t fillColor, uint16_t emptyColor) 
{
    int cx = 120, cy = 120;  // Center of the round display (assuming 240x240)
    int rOuter = 120;        // Outer radius of the arc
    int rInner = rOuter - 20; // Inner radius (defines thickness)

    // Angle range based on left or right side placement

    float startAngle ; 
    float endAngle   ;
    float filledAngle;

    if( isLeft )
    {
      startAngle  = 90; 
      endAngle    = startAngle + 180.0;
      filledAngle = map(value, 100, 0, endAngle, startAngle);

      // Draw the filled portion of the arc
      gfx->fillArc(cx, cy, rOuter, rInner, filledAngle, endAngle, emptyColor);
      // Draw the empty arc first
      gfx->fillArc(cx, cy, rOuter, rInner, startAngle, filledAngle, fillColor);
    }
    else
    {
      startAngle  = 270; 
      endAngle    = startAngle + 180.0;
      filledAngle = map(value, 0, 100, endAngle, startAngle);

      // Draw the filled portion of the arc
      gfx->fillArc(cx, cy, rOuter, rInner, filledAngle, endAngle, fillColor);
      // Draw the empty arc first
      gfx->fillArc(cx, cy, rOuter, rInner, startAngle, filledAngle, emptyColor);
    }
}

int prevBrakeSlider = 0;
void LCDDisplay::drawBrakeSlider(int value, uint16_t fillColor, uint16_t emptyColor) 
{
    int cx = 120, cy = 120;  // Center of the round display (assuming 240x240)
    int rOuter = 100;        // Outer radius of the arc
    int rInner = 80; // Inner radius (defines thickness)
    int rFilled = map(value, 0, 100, rOuter, rInner);
    
    if( prevBrakeSlider != rFilled)
    {
      gfx->fillArc(cx, cy, rInner, rFilled, 0, 360, emptyColor);
      gfx->fillArc(cx, cy, rFilled, rOuter, 0, 360, fillColor);
    }

    prevBrakeSlider = rFilled;
}



// Draws a horizontal slider
void LCDDisplay::drawSlider(int x, int y, int value, const char* label)
{
    gfx->drawRect(x, y, 80, 8, WHITE);
    gfx->fillRect(x+1, y+1, 78, 6, BLACK);
    gfx->fillRect(x+1, y+1, map(value, 0, 100, 0, 80)-2, 6, BLUE);
    gfx->setCursor(x + 85, y);
    gfx->print(label);
}


// Draws an arrow for inclination angle
void LCDDisplay::drawInclinationArrow(int x, int y, float angle)
{
    static float prev_angle = 0;

    float rad = prev_angle * DEG_TO_RAD;
    int length = 10;
    int arrowX = x + cos(rad) * length;
    int arrowY = y + sin(rad) * length;
    gfx->drawLine(x, y, arrowX, arrowY, BLACK);

    rad = angle * DEG_TO_RAD;
    length = 10;
    arrowX = x + cos(rad) * length;
    arrowY = y + sin(rad) * length;
    gfx->drawLine(x, y, arrowX, arrowY, YELLOW);

    prev_angle = angle;
}