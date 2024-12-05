/*******************************************************************************
 ESP32-S2 various dev board  : CS: 34, DC: 38, SCK: 36, MOSI: 35
 ESP32-S3 various dev board  : CS: 40, DC: 41, SCK: 36, MOSI: 35 
 ******************************************************************************/

#include <Arduino_GFX_Library.h>

Arduino_DataBus *bus = new Arduino_ESP32SPI(18 /* DC */, 16 /* CS */,  36/* SCL - SCK */, 35 /* SDA - MOSI */, -1);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, -1, 0 /* rotation */, true /* IPS */);

void setup(void)
{
  Serial.begin(115200);
  Serial.println("Arduino_GFX Hello World example");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
  }

  gfx->fillScreen(BLACK);
  gfx->setCursor(10, 10);
  gfx->setTextColor(RED);
  gfx->println("Hello World!");

  delay(500); // 5 seconds
}

void loop()
{
  gfx->setCursor(random(gfx->width()), random(gfx->height()));
  gfx->setTextColor(random(0xffff), random(0xffff));
  gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
  gfx->println("Hello World!");

  delay(1000); // 1 second
}
