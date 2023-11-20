#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

// Use UART1
HardwareSerial SerialPort(1);

// Pins for UART1
const int SERIAL_TX_PIN = 9;
const int SERIAL_RX_PIN = 11;

// ST7735 display pins


#define TFT_SCK 36
#define TFT_SDA 21
#define TFT_A0   40
#define TFT_RESET  38 
#define TFT_CS   34



// Initialize Adafruit ST7735 with hardware SPI
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_A0, TFT_SDA, TFT_SCK, TFT_RESET);

// Button pin assignments
const int buttonPins[] = {3, 5, 7, 2, 4, 6};
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// Variables to track previous button states
int prevButtonStates[numButtons] = {0};

// Function to update the display with squares corresponding to button states
void updateDisplay(int buttonStates[]) {
  for (int i = 0; i < numButtons; i++) {
    if (buttonStates[i] != prevButtonStates[i]) {
      // Only update display if button state has changed
      int color = buttonStates[i] ? ST7735_WHITE : ST7735_BLACK;
      // Choose your square positions and size according to your display and preferences
      int x = i * 20; // Example x position
      int y = 50; // Example y position
      int w = 18; // Example width of square
      int h = 18; // Example height of square
      prevButtonStates[i] = buttonStates[i];
      // Update the specific square section for each button
      tft.fillRect(x, y, w, h, color);
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



