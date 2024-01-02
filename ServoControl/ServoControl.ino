#include <Servo.h>

Servo servoMotor;  // Define the servo object

const int servoPin = 9;  // Define the pin for the servo
bool servoAttached = true;  // Flag to track if the servo is attached

void setupServo() 
{
  servoMotor.attach(servoPin);  // Attaches the servo to the specified pin
  Serial.begin(9600);  // Initialize serial communication
}

int targetAngle = 0;

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

void setup() 
{
  setupServo();  // Call the method to set up the servo
}

double Voltage = 0;
double Current = 0;
float voltageDevideFactor = 0.0117;//12 / 1024; // Voltage DC / Arduino analogue precision

void loop() 
{
  readSerialInput();  // Call the method to read serial input


  Voltage = 0;
  for(int i = 0; i < 500; i++) 
  {
    Voltage +=  analogRead(A0); // (V DC/ 1024 (Analog) = 0.0049) which converter Measured analog input voltage to 5 V Range
    delay(1);
  }

  Voltage = Voltage  / 500;
  double convertedVoltage = Voltage * voltageDevideFactor;

  Current = (Voltage -2.5)/ 0.185; // Sensed voltage is converter to current

  Serial.print("Voltage:"); // shows the measured voltage
  Serial.print(Voltage,2); // the ‘2’ after voltage allows you to display 2 digits after decimal point
  Serial.print("convertedVoltage:"); // shows the measured voltage
  Serial.print(convertedVoltage,2); // the ‘2’ after voltage allows you to display 2 digits after decimal point
  Serial.print(", Current:"); // shows the voltage measured
  Serial.println(Current,2); // the ‘2’ after voltage allows you to display 2 digits after decimal point

  Voltage = 0;

}
