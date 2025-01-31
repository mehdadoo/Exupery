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



void LCDDisplay::update(
    bool button1, bool button2, bool button3, bool button4, 
    float speed, bool pedalSensorisStopped,
    int joystick_throttle, int joystick_knob, int joystick_steering,
    float voltage, int steeringPercentage, int brakePercentage, int throttle1_perentage, int throttle2_perentage,
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
    drawButtonIndicator(centerX - 30, 15, button1);
    drawButtonIndicator(centerX - 10, 15, button2);
    drawButtonIndicator(centerX + 10, 15, button3);
    drawButtonIndicator(centerX + 30, 15, button4);

    // Draw Sliders
    drawTextBox(60, 30, joystick_throttle, "Throttle");
    drawTextBox(60, 50, joystick_knob, "Knob");
    drawTextBox(60, 70, joystick_steering, "Joystick S");
    drawTextBox(60, 90, steeringPercentage, "steering");
    drawTextBox(60, 110, brakePercentage, "brake");
    drawTextBox(60, 130, throttle1_perentage, "throttle1");
    drawTextBox(60, 150, throttle2_perentage, "throttle2");
    drawTextBox(60, 170, speed, "Speed");
    drawTextBox(60, 190, voltage, "voltage");

    drawButtonIndicator(gfx->width() - 30, centerY- 50, pedalSensorisStopped);

    // Draw Inclination Arrow
    drawInclinationArrow(centerX, 210, inclinationAngle);
}

// ======================== DRAW HELPER FUNCTIONS ========================

void LCDDisplay::drawTextBox(int x, int y, float value, const char* label) 
{
    // Define a fixed-size box for text clearing
    int textWidth = 100;  // Adjust width as needed
    int textHeight = 20; // Adjust height as needed

    // Erase previous value by drawing a black rectangle
    gfx->fillRect(x, y, textWidth, textHeight, BLACK);

    // Draw the new text
    gfx->setTextColor(WHITE);
    gfx->setCursor(x + 5, y + 5); // Small padding inside the box

    // Correct way to concatenate C-string with String object
    gfx->print(String(label) + ": " + String(value, 1));
}



// Draws small LED indicators for buttons
void LCDDisplay::drawButtonIndicator(int x, int y, bool state)
{
    uint16_t color = state ? GREEN : RED;
    gfx->fillCircle(x, y, 5, color);
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
    int length = 30;
    int arrowX = x + cos(rad) * length;
    int arrowY = y + sin(rad) * length;
    gfx->drawLine(x, y, arrowX, arrowY, BLACK);

    rad = angle * DEG_TO_RAD;
    length = 30;
    arrowX = x + cos(rad) * length;
    arrowY = y + sin(rad) * length;
    gfx->drawLine(x, y, arrowX, arrowY, YELLOW);

    prev_angle = angle;
}