#include <QMC5883LCompass.h>
#include <Wire.h>

//I2c Pins
#define I2C_SDA 18
#define I2C_SCL 16
// Pins for UART1
#define SERIAL_TX_PIN 35 // <- receive pin
#define SERIAL_RX_PIN 33 // -> send pin, connects to target RX

// Use UART1
HardwareSerial SerialPort(1);

TwoWire customWire = TwoWire(0);
QMC5883LCompass compass = QMC5883LCompass(&customWire);

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);

  delay(4000);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);

  SerialPort.print("START|v6|END");
  Serial.println("v10");


  customWire.begin( I2C_SDA, I2C_SCL, 100000 );

  Serial.println("customWire.begin");
  delay(3000);

  String result = compass.init();
  
  Serial.println("compass.init()");

  if (result == "Success") 
  {
      Serial.println("Sensor initialized successfully");
  }
  else
  {
      Serial.print("Error initializing sensor: ");
      Serial.println(result);
  }
}

void loop() 
{
  SerialPort.print("START|loop|END");
  Serial.println("loop");

 
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
