
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

/* more fonts at: https://github.com/moononournation/ArduinoFreeFontFile.git */
#include "FreeMono8pt7b.h"
#include "FreeSansBold10pt7b.h"
#include "FreeSerifBoldItalic12pt7b.h"

void setup(void)
{
  Serial.begin(115200);
  // Serial.setDebugOutput(true);
  // while(!Serial);
  Serial.println("Arduino_GFX Hello World Gfxfont example");

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

  gfx->setCursor(10, 10);
  gfx->setFont(&FreeMono8pt7b);
  gfx->setTextColor(RED);
  gfx->println("Hello World!");

  delay(5000); // 5 seconds
}

void loop()
{
  gfx->setCursor(random(gfx->width()), random(gfx->height()));
  gfx->setTextColor(random(0xffff));
  uint8_t textSize = random(3);
  switch (textSize)
  {
  case 1:
    gfx->setFont(&FreeMono8pt7b);
    break;
  case 2:
    gfx->setFont(&FreeSansBold10pt7b);
    break;
  default:
    gfx->setFont(&FreeSerifBoldItalic12pt7b);
    break;
  }

  gfx->println("Hello World!");

  delay(1000); // 1 second
}
