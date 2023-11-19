#include <HardwareSerial.h>

// Use UART1
HardwareSerial SerialPort(1); 

// Pins for UART1
const int SERIAL_TX_PIN = 9;
const int SERIAL_RX_PIN = 11;

// Pins for buttons
const int BUTTON_3_PIN = 3;
const int BUTTON_5_PIN = 5;
const int BUTTON_7_PIN = 7;
const int BUTTON_36_PIN = 36;
const int BUTTON_38_PIN = 38;
const int BUTTON_40_PIN = 40;

// Variables to track previous button states
int prevButton3State = 0;
int prevButton5State = 0;
int prevButton7State = 0;
int prevButton36State = 0;
int prevButton38State = 0;
int prevButton40State = 0;

// Function to handle the main logic of checking button states and sending data
void handleButtonStates() {
  // Read the state of buttons
  int button3State = !digitalRead(BUTTON_3_PIN);
  int button5State = !digitalRead(BUTTON_5_PIN);
  int button7State = !digitalRead(BUTTON_7_PIN);
  int button36State = !digitalRead(BUTTON_36_PIN);
  int button38State = !digitalRead(BUTTON_38_PIN);
  int button40State = !digitalRead(BUTTON_40_PIN);

  // Check if any button state has changed
  if (prevButton3State != button3State || prevButton5State != button5State || prevButton7State != button7State || prevButton36State != button36State || prevButton38State != button38State || prevButton40State != button40State) {
    // Creating the message string
    String message = "START";

    // Append the changed button states to the message
    if (prevButton3State != button3State) {
      message += "|B3:";
      message += button3State;
    }
    if (prevButton5State != button5State) {
      message += "|B5:";
      message += button5State;
    }
    if (prevButton7State != button7State) {
      message += "|B7:";
      message += button7State;
    }
    if (prevButton36State != button36State) {
      message += "|B36:";
      message += button36State;
    }
    if (prevButton38State != button38State) {
      message += "|B38:";
      message += button38State;
    }
    if (prevButton40State != button40State) {
      message += "|B40:";
      message += button40State;
    }

    message += "|END";

    // Print the message to Serial Monitor
    Serial.println(message);

    // Send the message over UART
    SerialPort.print(message);

    // Update previous button states
    prevButton3State = button3State;
    prevButton5State = button5State;
    prevButton7State = button7State;
    prevButton36State = button36State;
    prevButton38State = button38State;
    prevButton40State = button40State;
  }
}

void setup() 
{
  Serial.begin(115200);
  
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_5_PIN, INPUT_PULLUP);
  pinMode(BUTTON_7_PIN, INPUT_PULLUP);
  pinMode(BUTTON_36_PIN, INPUT_PULLUP);
  pinMode(BUTTON_38_PIN, INPUT_PULLUP);
  pinMode(BUTTON_40_PIN, INPUT_PULLUP);

  SerialPort.begin(115200, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);
}

void loop() 
{
  handleButtonStates(); // Function call for handling button states

  delay(25); // Delay to avoid excessive looping
}
