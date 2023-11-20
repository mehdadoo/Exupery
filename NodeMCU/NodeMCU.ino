#include <U8g2lib.h>
#include <ESP8266WiFi.h>

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

String receivedMsg = "";
bool messageStarted = false;

// Array to hold button states
bool buttonStates[6] = {false, false, false, false, false, false};

// Variables for display timing
unsigned long lastDisplayUpdateTime = 0;
const unsigned long displayUpdateInterval = 250; // Update display every 250 milliseconds

void setup() 
{
  u8g2.begin();
  Serial.begin(115200);
}

void loop() 
{
  receiveAndParseMessage();

  unsigned long currentMillis = millis();
  // Update display at regular intervals
  if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval) 
  {
    drawSquares();
    lastDisplayUpdateTime = currentMillis;
  }
}

// Function to receive and parse incoming messages
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

// Function to parse the received message and update button states
void parseMessage() 
{
  if (receivedMsg != "") 
  {
    Serial.println(receivedMsg);

    // Remove START and END markers
    receivedMsg.replace("START|", "");
    receivedMsg.replace("|END", "");

    // Process the message to extract key-value pairs
    int pos = 0;
    while (pos < receivedMsg.length()) 
    {
      String key;
      int value;

      // Extract key (B3, B5, etc.)
      int keyStart = receivedMsg.indexOf("B", pos);
      int keyEnd = receivedMsg.indexOf(":", keyStart);
      if (keyStart != -1 && keyEnd != -1) 
      {
        key = receivedMsg.substring(keyStart, keyEnd + 1);

        // Extract value
        int nextKeyPos = receivedMsg.indexOf("B", keyEnd);
        if (nextKeyPos == -1) 
        {
          value = receivedMsg.substring(keyEnd + 1).toInt();
          pos = receivedMsg.length(); // Exit loop if it's the last value
        } 
        else 
        {
          value = receivedMsg.substring(keyEnd + 1, nextKeyPos).toInt();
          pos = nextKeyPos;
        }

        // Assign value based on key

        if (key == "B3:") buttonStates[0] = value;
        else if (key == "B5:") buttonStates[1] = value;
        else if (key == "B7:") buttonStates[2] = value;
        else if (key == "B2:") buttonStates[3] = value;
        else if (key == "B4:") buttonStates[4] = value;
        else if (key == "B6:") buttonStates[5] = value;


      }
    }

    // Reset received message after processing
    messageStarted = false;
    receivedMsg = "";
  }
}

// Function to draw squares based on button states
void drawSquares() 
{
  u8g2.clearBuffer();

  for (int i = 0; i < 6; i++) {
    if (buttonStates[i]) {
      u8g2.drawBox(i * 20, 0, 10, 10);
    }
  }

  u8g2.sendBuffer();
}