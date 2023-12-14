#include <ESP8266WiFi.h>

String receivedMsg = "";
bool messageStarted = false;

void setup() 
{
  Serial.begin(9600);
}

void loop() {
  receiveMessage();
  parseMessage();
}

// Function to receive incoming messages over UART
void receiveMessage() 
{
  while (Serial.available()) {
    char character = Serial.read();
    receivedMsg.concat(character);
  }
}

// Function to parse incoming message and display it on the serial monitor
void parseMessage() 
{
  if (receivedMsg.startsWith("START")) 
    messageStarted = true;

  if (receivedMsg.endsWith("END")) {
    if (receivedMsg != "") 
    {
      // Remove START and END markers
      receivedMsg.replace("START|", "");
      receivedMsg.replace("|END", "");
      Serial.println(receivedMsg);
      Serial.println("");
    }
    // Reset variables after processing
    messageStarted = false;
    receivedMsg = "";
  }
}
