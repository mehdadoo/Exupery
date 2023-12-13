/*
===============================================================================================================
QMC5883LCompass.h Library XYZ Example Sketch
Learn more at [https://github.com/mprograms/QMC5883LCompass]

This example shows how to get the XYZ values from the sensor.

===============================================================================================================
Release under the GNU General Public License v3
[https://www.gnu.org/licenses/gpl-3.0.en.html]
===============================================================================================================
*/
#include <QMC5883LCompass.h>
#include <Wire.h>

//I2c Pins
#define I2C_SDA 8
#define I2C_SCL 9
// Pins for UART1
#define SERIAL_TX_PIN 38
#define SERIAL_RX_PIN 40

// Use UART1
HardwareSerial SerialPort(1);

TwoWire customWire = TwoWire(0);
QMC5883LCompass compass(customWire);


void setup() 
{
   pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);

  customWire.begin( I2C_SDA, I2C_SCL );
  compass.init();

  SerialPort.print("START|Compass|END");
}

void loop() 
{
  int x, y, z;
  
  // Read compass values
  compass.read();

  // Return XYZ readings
  x = compass.getX();
  y = compass.getY();
  z = compass.getZ();
  
  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.print(z);
  Serial.println();

  SerialPort.print("START|" + String(x) + ", " + String(y) + ", " + String(z) + "|END");

  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
}
