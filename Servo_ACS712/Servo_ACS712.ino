#include <Servo.h>
#include "ACS712.h"

Servo servoMotor;  // Define the servo object
const int servoPin = 9;  // Define the pin for the servo
bool servoAttached = true;  // Flag to track if the servo is attached

ACS712 ACS(A0, 5.0, 1023, 100);  // Initialize ACS712 current sensor

int targetAngle = 0;

void setupServo() 
{
  servoMotor.attach(servoPin);  // Attaches the servo to the specified pin
}

void setup()
{
  setupServo();  // Call the method to set up the servo

  Serial.begin(115200);
  while (!Serial);
  Serial.println(__FILE__);
  Serial.print("ACS712_LIB_VERSION: ");
  Serial.println(ACS712_LIB_VERSION);

  ACS.autoMidPoint();
  //Serial.println(ACS.getMidPoint());
}

void readSerialInput() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');  // Read the input from serial monitor
    targetAngle = input.toInt();  // Get the target angle

    if (!servoAttached) 
    {
      //servoAttached = true;
      //servoMotor.attach(servoPin);  // Reattach the servo if it was detached
      //delay(100);
    }

    servoMotor.write(targetAngle);  // Move the servo to the target angle
  }
}

void loop()
{
  readSerialInput();  // Call the method to read serial input

  int mA = ACS.mA_DC(1024);  // Measure current in mA
  Serial.println(mA);  // Output current measurement to serial monitor
}
