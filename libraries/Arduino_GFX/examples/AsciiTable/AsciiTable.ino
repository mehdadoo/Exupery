
/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/
#include "Arduino_DataBus.h"
#include "databus/Arduino_ESP32SPI.h"
#include "databus/Arduino_Wire.h"
#include "Arduino_GFX.h"
#include "display/Arduino_GC9A01.h"

#define TFT_CS 34 
#define TFT_DC 38
#define TFT_RST 33
#define GFX_BL 21
#define TFT_SCK 36
#define TFT_MOSI 35
#define TFT_MISO -1

Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, TFT_MISO, HSPI /* spi_num */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

void setup(void)
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX AsciiTable example");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);

#ifdef GFX_BL
  pinMode(GFX_BL, OUTPUT);
  digitalWrite(GFX_BL, HIGH);
#endif

  gfx->setTextColor(GREEN);
  for (int x = 0; x < 16; x++)
  {
    gfx->setCursor(10 + x * 8, 2);
    gfx->print(x, 16);
  }
  gfx->setTextColor(BLUE);
  for (int y = 0; y < 16; y++)
  {
    gfx->setCursor(2, 12 + y * 10);
    gfx->print(y, 16);
  }

  char c = 0;
  for (int y = 0; y < 16; y++)
  {
    for (int x = 0; x < 16; x++)
    {
      gfx->drawChar(10 + x * 8, 12 + y * 10, c++, WHITE, BLACK);
    }
  }

  delay(5000); // 5 seconds
}

void loop()
{
}