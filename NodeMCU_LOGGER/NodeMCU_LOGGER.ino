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
  while (Serial.available()) 
  {
    char character = Serial.read();
    receivedMsg.concat(character);
  }
}

// Function to parse incoming message and display it on the serial monitor
void parseMessage() 
{
  if (receivedMsg == "") 
    return;

  int endIndex = receivedMsg.lastIndexOf("|END");
  if (endIndex == -1) 
    return;

  int startIndex = receivedMsg.lastIndexOf("START|", endIndex);
  if (startIndex == -1) 
    return;

  // Extract the valid message between START| and |END
  String validMessage = receivedMsg.substring(startIndex + 6, endIndex);

  // Print the valid message
  Serial.println(validMessage);

  // Clear processed message
  receivedMsg = "";

  // Reset variables after processing
  messageStarted = false;
}
