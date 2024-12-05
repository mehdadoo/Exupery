#include <ESP32Servo.h>

Servo servoMotor;  // Define the servo object
const int servoPin = 17;  // Define the pin for the servo
bool servoAttached = true;  // Flag to track if the servo is attached

int targetAngle = 0;

void setupServo() 
{
  servoMotor.attach(servoPin);  // Attaches the servo to the specified pin
  Serial.begin(9600);  // Initialize serial communication
}

void setup()
{
  setupServo();  // Call the method to set up the servo
  Serial.begin(115200);
  while (!Serial);
}

void readSerialInput() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');  // Read the input from serial monitor
    targetAngle = input.toInt();  // Get the target angle

    if (!servoAttached) 
    {
      servoAttached = true;
      servoMotor.attach(servoPin);  // Reattach the servo if it was detached
      delay(100);
    }

    servoMotor.write(targetAngle);  // Move the servo to the target angle
  }
}

void loop()
{
  readSerialInput();  // Call the method to read serial input
  delay(100);  // Adjust delay as needed
}
