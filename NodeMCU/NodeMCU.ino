#include <U8g2lib.h>
#include <ESP8266WiFi.h>

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);


String receivedMsg = "";
bool messageStarted = false;

// Array to hold button states
const int numButtons = 7;

bool buttonStates[numButtons] = {false};
bool prevButtonStates[numButtons] = {false}; 

// Variables for joystick values
int joystickX = 0;
int joystickY = 0;

// Variables for display timing
unsigned long lastDisplayUpdateTime = 0;
const unsigned long displayUpdateInterval = 250; // Update display every 250 milliseconds


// Pin for Horn MOSFET control
const int mosfetPin_Horn = D2;
const int mosfetPin_BrakeLight = D3;


void setup() 
{
  u8g2.begin();
  Serial.begin(115200);

  
  pinMode(mosfetPin_Horn, OUTPUT); // Set MOSFET pin as output
  pinMode(mosfetPin_BrakeLight, OUTPUT); // Set MOSFET pin as output
}

void loop() 
{
  receiveMessage();
  parseMessage();

  unsigned long currentMillis = millis();
  // Update display at regular intervals
  if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval) 
  {
    updateOLED();
    lastDisplayUpdateTime = currentMillis;
  }

  controlMOSFET(); // Function call to control the MOSFET
}

// Function to receive and parse incoming messages
void receiveMessage() 
{
  while (Serial.available()) 
  {
    char character = Serial.read();
    receivedMsg.concat(character);
  }
}
void parseMessage() 
{
  if (receivedMsg.startsWith("START")) 
    messageStarted = true;

  if (messageStarted && receivedMsg.endsWith("END")) 
    decodeMessage();
}


// Function to parse the received message and update button states
void decodeMessage() 
{
  if (receivedMsg == "") 
    return;
  
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

      if (key == "B0:") buttonStates[0] = value;
      else if (key == "B1:") buttonStates[1] = value;
      else if (key == "B2:") buttonStates[2] = value;
      else if (key == "B3:") buttonStates[3] = value;
      else if (key == "B4:") buttonStates[4] = value;
      else if (key == "B5:") buttonStates[5] = value;
      else if (key == "B6:") buttonStates[6] = value;
      else if (key == "BX:") joystickX = value;
      else if (key == "BY:") joystickY = value;
    }
  }

  // Reset received message after processing
  messageStarted = false;
  receivedMsg = "";
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


int pixel_x = 0;
int pixel_y = 0;

int square_x = 0; //  x position
int square_y = 16; //  y position
int square_width = 48; //  width of square
int joystick_maxValue = 3500;

void updateOLED() 
{
 // Clear the internal memory
  for (int i = 0; i < numButtons; i++) 
  {
    // Only update the part of the screen that has changed
    if (buttonStates[i] != prevButtonStates[i]) 
    {
      prevButtonStates[i] = buttonStates[i]; // Update previous state
      
      // Example values, adjust based on your screen layout and preferences
      int x = i * 20; // Position for squares
      int y = 0; 
      int w = 10; // Size of squares
      int h = 10;
      
      if (buttonStates[i]) 
      {
        u8g2.setDrawColor(1);
        u8g2.drawBox(x, y, w, h); // Draw a filled square
      }
      else 
      {
        u8g2.setDrawColor(0);
        u8g2.drawBox(x, y, w, h); // Draw an empty square
      }
    }
  }

  
  // clear drawn pixel only
   u8g2.setDrawColor(0);
   u8g2.drawPixel( pixel_x, pixel_y);

  pixel_x =  map(joystickX, joystick_maxValue, 0, square_x, square_width + square_x) ;
  pixel_y =  map(joystickY, 0, joystick_maxValue, square_y, square_width + square_y);

  u8g2.setDrawColor(1);
  u8g2.drawPixel( pixel_x, pixel_y);

  u8g2.drawFrame(square_x, square_y, square_width, square_width);

  u8g2.sendBuffer(); // Transfer internal memory to the display
}



// Function to control MOSFET based on received message
void controlMOSFET() 
{
  if (buttonStates[2]) 
  { 
    // Check the value of B2 received from the sender
    digitalWrite(mosfetPin_Horn, HIGH); // Set MOSFET pin high
  } 
  else 
  {
    digitalWrite(mosfetPin_Horn, LOW); // Set MOSFET pin low
  }

  if (buttonStates[3]) 
  { 
    // Check the value of B2 received from the sender
    digitalWrite(mosfetPin_BrakeLight, HIGH); // Set MOSFET pin high
  } 
  else 
  {
    digitalWrite(mosfetPin_BrakeLight, LOW); // Set MOSFET pin low
  }
}