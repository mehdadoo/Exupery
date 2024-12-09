#include <ESP32Servo.h>

Servo servoMotor;  // Define the servo object
const int servoPin = 17;  // Define the pin for the servo
int targetAngle = 0;

void setup()
{
  Serial.begin(9600);

  servoMotor.attach(servoPin);  // Reattach the servo if it was detached
  delay(100);

  Serial.println("servo app");
}

void readSerialInput() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');  // Read the input from serial monitor
    targetAngle = input.toInt();  // Get the target angle
    Serial.println("servo:"+ input);
    servoMotor.write(targetAngle);  // Move the servo to the target angle

   
  }
}

void loop()
{
  readSerialInput();  // Call the method to read serial input
  delay(100);  // Adjust delay as needed
}
