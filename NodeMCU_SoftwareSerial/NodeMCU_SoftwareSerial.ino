#include <U8g2lib.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

SoftwareSerial NodeMCU(D5,D6);

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

void setup() {
  u8g2.begin();

 
  NodeMCU.begin(115200);

  pinMode(D5,INPUT);
  pinMode(D6,OUTPUT);

  Serial.begin(115200); 
}

void loop() 
{
  String content = "";
  char character;

  while (NodeMCU.available()) 
  {
    character = NodeMCU.read();
    content.concat(character);
  }

  if (content != "") 
  {
    Serial.println(content); // Print the received message

    u8g2.clearBuffer();		
    u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
    u8g2.drawStr(0,10, content.c_str());	// write something to the internal memory
    u8g2.sendBuffer();	  // transfer internal memory to the display	
  }

  delay(500); // Add a small delay to avoid excessive looping
}