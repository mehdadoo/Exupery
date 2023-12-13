// include the library code:
#include <LiquidCrystal.h>
#include <QMC5883LCompass.h>
#include <Wire.h>

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
const int rs = 11, en = 10, d4 = 9, d5 = 8, d6 = 7, d7 = 6;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

QMC5883LCompass compass;

void setup() 
{
  Serial.begin(9600);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  lcdPrint( "Exupery3", 0, 0);

  /* Initialise the sensor */
  compass.init();
  delay(500);
}

void lcdPrint(String text, uint cursorX, uint cursorY) 
{
  lcd.setCursor(cursorX, cursorY);
  // Print a message to the LCD.
  lcd.print(text);
  Serial.println(text); // Print to Serial Monitor
}

bool blink = false;

void loop() 
{
  lcdPrint(blink == true ? "." : "_", 0, 1);

  blink = !blink; // Corrected toggle operation
  delay(500);
  

  int x, y, z;

  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();

  lcdPrint("x:" + String(x), 1, 1);
  delay(250);
  
}

