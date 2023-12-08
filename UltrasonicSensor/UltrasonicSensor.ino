#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>

#define trigPin D2
#define echoPin D3
 
long duration;
int distance;
 
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

void setup() 
{
  Serial.begin(9600);
  u8g2.begin();

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

  u8g2.clearBuffer();	
  u8g2.setFont(u8g2_font_7x14B_tr);
  u8g2.drawStr(10, 10, "distance:" );
  u8g2.drawStr(10, 30, String(distance ).c_str()  );
  u8g2.sendBuffer();

  delay(200);
}