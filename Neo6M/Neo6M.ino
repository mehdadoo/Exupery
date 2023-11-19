#include<SoftwareSerial.h>
SoftwareSerial Neo6M(10, 11);

void setup() 
{
  Serial.begin(9600);
  Neo6M.begin(9600);
}

void loop()
 {
  while( Neo6M.available() > 0)
  {
    Serial.write( Neo6M.read());
  }

}
