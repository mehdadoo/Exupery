// Include the library
#include <Servo.h>


// Create the servo object
Servo myservo;

int speed = 80;
// Setup section to run once
void setup() {
  myservo.attach(3); // attach the servo to our servo object
  myservo.write(speed); // stop the motor
}

// Loop to keep the motor turning!
void loop() 
{
  if( digitalRead(6) )
  speed +=5;

  if( digitalRead(7) )
  speed -=5;
}