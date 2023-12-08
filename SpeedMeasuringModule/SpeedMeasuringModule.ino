#include <LiquidCrystal.h>

const int rs = 5, en = 6, d4 = 7, d5 = 8, d6 = 9, d7 = 10;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

volatile byte state = LOW;

int pedalSpeedSensorPin= 2;

unsigned int rpm;
volatile byte pulses;
unsigned long time_old;
unsigned int pulses_per_turn= 20; // Depends on the number of spokes on the encoder wheel

void count() 
{
  pulses++; 
  blink();
}

void blink() 
{
  state = !state;
}

void setup() 
{
 

  rpm=0;
  pulses=0;
  time_old=0;
  
  Serial.begin(9600);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(pedalSpeedSensorPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(pedalSpeedSensorPin), count, FALLING);

  //setupLCD();
}

void loop() 
{
  digitalWrite(LED_BUILTIN, state);

  return;
  if(millis()-time_old >=100)
  { // Updating every 0.1 seconds
    detachInterrupt(digitalPinToInterrupt(pedalSpeedSensorPin));
    rpm = (60 * 100 / pulses_per_turn )/ (millis() - time_old)* pulses;
    time_old=millis();
    pulses=0;
    Serial.print("RPM= ");
    Serial.println(rpm);
    attachInterrupt(digitalPinToInterrupt(pedalSpeedSensorPin), count, FALLING);
  }
}



void setupLCD() 
{
  lcd.begin(16, 2);
  lcd.print("First line");
  lcd.setCursor(0,1);
  lcd.print("Second line");
}
