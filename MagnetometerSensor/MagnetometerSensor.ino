#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//I2c Pins
#define I2C_SDA 18
#define I2C_SCL 16
// Pins for UART1
#define SERIAL_TX_PIN 35
#define SERIAL_RX_PIN 33

// Use UART1
HardwareSerial SerialPort(1);

//I2C custom Wire
TwoWire custom_i2c_bus  = TwoWire(0);
Adafruit_HMC5883_Unified magnetometer = Adafruit_HMC5883_Unified(&custom_i2c_bus );

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);

  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);

  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(50);

  SerialPort.print("START|v1|END");
  Serial.println("v1");
  
  custom_i2c_bus .begin(I2C_SDA, I2C_SCL, 100000);
  delay(1000);

  if(!magnetometer.begin(&custom_i2c_bus ))
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  else
  {
    Serial.println("HMC5883 detected");
  }
}

void loop()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  magnetometer.getEvent(&event);
  Serial.println(event.magnetic.z);

 

  digitalWrite(LED_BUILTIN, HIGH);
  delay(150);
  digitalWrite(LED_BUILTIN, LOW);
  delay(450);
}
