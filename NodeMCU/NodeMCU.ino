#include <U8g2lib.h>
#include <ESP8266WiFi.h>

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

String receivedMsg = "";
bool messageStarted = false;

bool b3 = false;
bool b5 = false;
bool b7 = false;
bool b36 = false;
bool b38 = false;
bool b40 = false;

void setup() 
{
  u8g2.begin();
  Serial.begin(115200);
}

void loop() 
{
  receiveAndParseMessage();
  drawSquares();
}

void receiveAndParseMessage() 
{
  while (Serial.available()) 
  {
    char character = Serial.read();
    receivedMsg.concat(character);

    if (receivedMsg.endsWith("START")) 
    {
      messageStarted = true;
    }

    if (messageStarted && receivedMsg.endsWith("END")) 
    {
      parseMessage();
      break;
    }
  }
}

void parseMessage() 
{
  if (receivedMsg != "") 
  {
    // Remove START and END markers
    receivedMsg.replace("START|", "");
    receivedMsg.replace("|END", "");

    // Extract values for B3, B5, B7, B36, B38, B40
    b3 = receivedMsg.substring(receivedMsg.indexOf("B3:") + 3, receivedMsg.indexOf("|B5:")).toInt();
    b5 = receivedMsg.substring(receivedMsg.indexOf("B5:") + 3, receivedMsg.indexOf("|B7:")).toInt();
    b7 = receivedMsg.substring(receivedMsg.indexOf("B7:") + 3, receivedMsg.indexOf("|B36:")).toInt();
    b36 = receivedMsg.substring(receivedMsg.indexOf("B36:") + 4, receivedMsg.indexOf("|B38:")).toInt();
    b38 = receivedMsg.substring(receivedMsg.indexOf("B38:") + 4, receivedMsg.indexOf("|B40:")).toInt();
    b40 = receivedMsg.substring(receivedMsg.indexOf("B40:") + 4).toInt();

    // Reset received message after processing
    messageStarted = false;
    receivedMsg = "";
  }
}

 void drawSquares() 
  {
    u8g2.clearBuffer();

    if (!b3)
      u8g2.drawBox(0, 0, 10, 10);
    if (!b5)
      u8g2.drawBox(20, 0, 10, 10);
    if (!b7)
      u8g2.drawBox(40, 0, 10, 10);
    if (!b36)
      u8g2.drawBox(60, 0, 10, 10);
    if (!b38)
      u8g2.drawBox(80, 0, 10, 10);
    if (!b40)
      u8g2.drawBox(100, 0, 10, 10);

    u8g2.sendBuffer();
  }
