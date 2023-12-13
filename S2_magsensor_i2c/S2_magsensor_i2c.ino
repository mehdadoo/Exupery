#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

//I2c Pins
#define I2C_SDA 8
#define I2C_SCL 9
// Pins for UART1
#define SERIAL_TX_PIN 38
#define SERIAL_RX_PIN 39

// Use UART1
HardwareSerial SerialPort(1);

//I2C custom Wire
TwoWire I2C_magsensor = TwoWire(0);

/* Assign a unique ID to this sensor at the same time */
Adafruit_HMC5883_Unified magsensor = Adafruit_HMC5883_Unified(&I2C_magsensor,12345);



void setup() 
{
  Serial.begin(9600);
  SerialPort.begin(9600, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);

  pinMode(LED_BUILTIN, OUTPUT);
  
  I2C_magsensor.begin(I2C_SDA, I2C_SCL, 100000);
  delay(3000);

  if(!magsensor.begin(&I2C_magsensor))
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  else
  {
    displaySensorDetails();
  }
}

void loop()
{
  /* Get a new sensor event */ 
  sensors_event_t event; 
  magsensor.getEvent(&event);
 
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  Serial.print("X: "); Serial.print(event.magnetic.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.magnetic.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.magnetic.z); Serial.print("  ");Serial.println("uT");

  // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.22;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  
  Serial.print("Heading (degrees): "); Serial.println(headingDegrees);

  //SerialPort.print("START|A:1|END");
  
  digitalWrite(LED_BUILTIN, HIGH);
  delay(50);
  digitalWrite(LED_BUILTIN, LOW);
  delay(200);
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  magsensor.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}
