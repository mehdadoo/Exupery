#include <SoftwareSerial.h>

// The serial connection to the GPS module
SoftwareSerial gps(3, 4);
SoftwareSerial sim(11, 10);

int _timeout;
String _buffer;
String number = "+33761789506"; //-> change with your number

void setup()
{
  Serial.begin(9600);
  Serial.println("Sistem Started...");
  gps.begin(9600);
  sim.begin(9600);


  _buffer.reserve(50);
  delay(1000);
  Serial.println("Type s to send an SMS, r to receive an SMS, and c to make a call");
}

void loop()
{
  if (Serial.available() > 0)
  {
      switch (Serial.read())
      {
        case 's':
          SendMessage();
          break;
      }
  }

  // get the byte data from the GPS
  while (gps.available() > 0)
    Serial.write( gps.read() );
  
  // get the byte data from the SIM800L
  while (sim.available() > 0)
    Serial.write(sim.read());

}



void SendMessage()
{
  Serial.println ("Sending Message");

  //Sets the GSM Module in Text Mode
  sim.println("AT+CMGF=1");    
  delay(200);

  //Mobile phone number to send message
  sim.println("AT+CMGS=\"" + number + "\"\r"); 
  delay(200);

  String SMS = "Hello, how are you? greetings from mehdadoo";
  sim.println(SMS);
  delay(100);

  sim.println((char)26);// ASCII code of CTRL+Z
  delay(200);

  _buffer = _readSerial();
  Serial.println(_buffer);
}


String _readSerial() 
{
  _timeout = 0;
  while  (!sim.available() && _timeout < 12000  )
  {
    delay(13);
    _timeout++;
  }
  if (sim.available()) 
  {
    return sim.readString();
  }
}