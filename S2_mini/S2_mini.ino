#include <HardwareSerial.h>

// Use UART1
HardwareSerial SerialPort(1);

// Pins for UART1
const int SERIAL_TX_PIN = 9;
const int SERIAL_RX_PIN = 11;

// Button pin assignments
const int buttonPins[] = {3, 5, 7, 2, 4, 6};
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// Variables to track previous button states
int prevButtonStates[numButtons] = {0};

// Function to handle the main logic of checking button states and sending data
void handleButtonStates() 
{
  bool stateChanged = false;
  String message = "START";

  for (int i = 0; i < numButtons; i++) 
  {
    int currentButtonState = !digitalRead(buttonPins[i]);
    // Check if button state has changed
    if (prevButtonStates[i] != currentButtonState) 
    {
      message += "|B" + String(buttonPins[i]) + ":" + String(currentButtonState);
      prevButtonStates[i] = currentButtonState;
      stateChanged = true;
    }
  }

  if (stateChanged) 
  {
    message += "|END";
    // Print the message to Serial Monitor
    Serial.println(message);
    // Send the message over UART
    SerialPort.print(message);
  }
}

void setup() 
{
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);

  // Initialize button pins as inputs with internal pull-up
  for (int i = 0; i < numButtons; i++) 
  {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
}

void loop() 
{
  handleButtonStates(); // Function call for handling button states
  delay(25); // Delay to avoid excessive looping
}