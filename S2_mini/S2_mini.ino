#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Use UART1
HardwareSerial SerialPort(1);

// Pins for UART1
const int SERIAL_TX_PIN = 9;
const int SERIAL_RX_PIN = 11;

// ST7735 display pins
#define TFT_CS   21
#define TFT_RST  34 
#define TFT_DC   36
#define TFT_MOSI 38  
#define TFT_SCLK 40  

// Initialize Adafruit ST7735 with hardware SPI
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_MOSI, TFT_SCLK, TFT_RST);

// Button pin assignments
const int buttonPins[] = {3, 5, 7, 2, 4, 6};
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// Variables to track previous button states
int prevButtonStates[numButtons] = {0};

// Function to update the display with squares corresponding to button states
void updateDisplay(const int buttonStates[]) {
  tft.fillScreen(ST7735_BLACK); // Clear the screen

  for (int i = 0; i < numButtons; i++) {
    if (buttonStates[i]) {
      // Draw a filled square for each pressed button
      tft.fillRect(i * 20, 50, 18, 18, ST7735_WHITE);
    }
  }
}

// Function to handle the main logic of checking button states and sending data
void handleButtonStates() {
  bool stateChanged = false;
  String message = "START";
  int buttonStates[numButtons] = {0};

  for (int i = 0; i < numButtons; i++) {
    buttonStates[i] = !digitalRead(buttonPins[i]);
    // Check if button state has changed
    if (prevButtonStates[i] != buttonStates[i]) {
      message += "|B" + String(buttonPins[i]) + ":" + String(buttonStates[i]);
      prevButtonStates[i] = buttonStates[i];
      stateChanged = true;
    }
  }

  if (stateChanged) {
    message += "|END";
    // Print the message to Serial Monitor
    Serial.println(message);
    // Send the message over UART
    SerialPort.print(message);
    // Update the display
    updateDisplay(buttonStates);
  }
}

void setup() {
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);

  // Initialize button pins as inputs with internal pull-up
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Initialize the ST7735 display
  tft.initR(INITR_BLACKTAB); // Init display with black tab
  tft.fillScreen(ST7735_BLACK); // Clear the screen
}

void loop() {
  handleButtonStates(); // Function call for handling button states
  delay(25); // Delay to avoid excessive looping
}