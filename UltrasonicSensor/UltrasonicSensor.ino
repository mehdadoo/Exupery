#include <Arduino.h>
#include <Wire.h>

#define trigPin 37
#define echoPin 38
 
long duration;
int distance;
 

void setup() 
{
  Serial.begin(9600);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  delay(2000);
}
 
void loop() 
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(20);
  digitalWrite(trigPin, LOW);
 
  duration = pulseIn(echoPin, HIGH);
  distance = duration*0.034/2.0;
 
  Serial.print("Distance = ");
  Serial.print(distance);
  Serial.println(" cm");

  delay(200);
}