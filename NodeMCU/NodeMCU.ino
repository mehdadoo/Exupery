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

void parseMessage() {
  if (receivedMsg != "") {
    // Remove START and END markers
    receivedMsg.replace("START|", "");
    receivedMsg.replace("|END", "");

    // Process the message
    int pos = 0;
    while (pos < receivedMsg.length()) {
      String key;
      int value;

      // Extract key (B3, B5, etc.)
      int keyStart = receivedMsg.indexOf("B", pos);
      int keyEnd = receivedMsg.indexOf(":", keyStart);
      if (keyStart != -1 && keyEnd != -1) {
        key = receivedMsg.substring(keyStart, keyEnd + 1);

        // Extract value
        int nextKeyPos = receivedMsg.indexOf("B", keyEnd);
        if (nextKeyPos == -1) {
          value = receivedMsg.substring(keyEnd + 1).toInt();
          pos = receivedMsg.length(); // Exit loop if it's the last value
        } else {
          value = receivedMsg.substring(keyEnd + 1, nextKeyPos).toInt();
          pos = nextKeyPos;
        }

        // Assign value based on key
        if (key == "B3:") {
          b3 = value;
        } else if (key == "B5:") {
          b5 = value;
        } else if (key == "B7:") {
          b7 = value;
        } else if (key == "B36:") {
          b36 = value;
        } else if (key == "B38:") {
          b38 = value;
        } else if (key == "B40:") {
          b40 = value;
        }
      }
    }

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
