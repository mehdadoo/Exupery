#include <U8g2lib.h>
#include <ESP8266WiFi.h>

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

String receivedMsg = "";
bool messageStarted = false;

void setup() 
{
  u8g2.begin();
  Serial.begin(115200);
}

void loop() 
{
  while (Serial.available()) 
  {
    char character = Serial.read();
    receivedMsg.concat(character);

    if ( receivedMsg.endsWith("START") )
    {
        messageStarted = true;
    }
    

    if ( messageStarted == true && receivedMsg.endsWith("END") )
    {
      parseMessage();
      break;
    }
  }

  // Small delay to avoid excessive looping
 
}

void parseMessage() 
{
  if( receivedMsg!= "" )
  {
      // Remove START and END markers
      receivedMsg.replace("START|", "");
      receivedMsg.replace("|END", "");

      // Print the complete received message
      Serial.println(receivedMsg); 
      u8g2.clearBuffer();		
      u8g2.setFont(u8g2_font_ncenB08_tr);
      u8g2.drawStr(0, 10, receivedMsg.c_str());
      u8g2.sendBuffer();

      // Reset received message after processing
      messageStarted = false;
      receivedMsg = ""; 
  }
}
