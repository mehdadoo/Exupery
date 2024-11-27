
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
  Serial.println("Arduino_GFX Set Text Bound example");

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

  gfx->drawRect(16, 16, 120, 90, WHITE);
  gfx->setTextBound(18, 18, 116, 86);
  // gfx->setTextWrap(false);
  // gfx->setTextSize(3, 3, 1);
  gfx->setCursor(30, 18);
  gfx->setTextColor(WHITE);
  gfx->println("Arduino has over the years released over 100 hardware products: boards, shields, carriers, kits and other accessories. In this page, you will find an overview of all active Arduino hardware, including the Nano, MKR and Classic families.");

  delay(5000); // 5 seconds
}

void loop()
{
  gfx->fillRect(18, 18, 116, 86, BLACK);
  gfx->setCursor(30, 18);
  gfx->setTextColor(random(0xffff), random(0xffff));
  gfx->setTextSize(random(6) /* x scale */, random(6) /* y scale */, random(2) /* pixel_margin */);
  gfx->println("Arduino has over the years released over 100 hardware products: boards, shields, carriers, kits and other accessories. In this page, you will find an overview of all active Arduino hardware, including the Nano, MKR and Classic families.");

  delay(1000); // 1 second
}
