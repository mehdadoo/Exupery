#include <QMC5883LCompass.h>
#include <Wire.h>

//I2c Pins
#define I2C_SDA 8
#define I2C_SCL 9
// Pins for UART1
#define SERIAL_TX_PIN 35 // <- receive pin
#define SERIAL_RX_PIN 33 // -> send pin, connects to target RX

// Use UART1
HardwareSerial SerialPort(1);

TwoWire customWire = TwoWire(0);
QMC5883LCompass compass(customWire);


void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);

 
 SerialPort.print("START|Compass version 2|END");
  

  customWire.begin( I2C_SDA, I2C_SCL );
  compass.init();

  
  
}

void loop() 
{
  SerialPort.print("START|version: 2|END");
 
 
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(250);


  return;
  /*int x, y, z;
  
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

  */
 
}
