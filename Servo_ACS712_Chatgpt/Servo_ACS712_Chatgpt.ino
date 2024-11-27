#include <Servo.h>

const uint16_t numReadings = 256;  // Number of readings to average
const uint8_t sensorPin = A0;     // Analog pin
uint16_t readings[numReadings];

int targetAngle = 0;

const uint8_t servoPin = 9;  // Define the pin for the servo
Servo servoMotor;  // Define the servo object

void setup() 
{
  Serial.begin(115200);
  servoMotor.attach(servoPin);  // Attaches the servo to the specified pin
}

void loop()
{
  readSerialInput();

  static uint16_t index = 0;  // Index for readings array
  static uint32_t sum = 0;    // Sum of readings
  static float prevVoltage = 0.0;  // Previous voltage value
  const float threshold = 0.01;  // Set your threshold value here

  // Read the input on the analog pin and store it in the array
  readings[index] = analogRead(sensorPin);
  sum += readings[index];  // Add the reading to the sum

  index++;  // Move to the next index
  if (index >= numReadings) 
  {
    // Calculate average voltage
    float averageVoltage = (sum * 5.0) / (1023.0 * numReadings);  // 5.0V is the reference voltage for Arduino Uno

    // Check if the change exceeds the threshold
    if (abs(averageVoltage - prevVoltage) > threshold) {
      // Print the average voltage
      Serial.print("2.595, ");
      Serial.println(averageVoltage, 3);  // Print with 4 decimal places

      // Update the previous voltage value
      prevVoltage = averageVoltage;
    }

    // Reset variables for the next set of readings
    index = 0;
    sum = 0;

    //delay(200);
  }
}


void readSerialInput() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');  // Read the input from serial monitor
    targetAngle = input.toInt();  // Get the target angle

    servoMotor.write(targetAngle);  // Move the servo to the target angle
  }
}
