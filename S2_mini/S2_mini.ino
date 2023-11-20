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
const int buttonPins[] = {3, 5, 2, 4, 6, 8};
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// Variables to track previous button states
int buttonStates[numButtons] = {0};
int prevButtonStates[numButtons] = {0};

// Joystick pins and last known values
const int joystickYPin = 7; // Joystick Y pin connected to GPIO 7
const int joystickXPin = 14; // Joystick X pin connected to GPIO 15
int lastJoystickX = -1; // Initialize with invalid value to make sure it sends the first reading
int lastJoystickY = -1; // Initialize with invalid value to make sure it sends the first reading
int smoothedJoystickX = 0; // Smoothed joystick X value
int smoothedJoystickY = 0; // Smoothed joystick Y value
const int numReadings = 10; // Number of readings to smooth joystick input
int joystickReadingsX[numReadings] = {0}; // Array to store joystick X readings
int joystickReadingsY[numReadings] = {0}; // Array to store joystick Y readings
unsigned int currentIndex = 0; // Index for readings arrays
unsigned int readingsCount = 0; // Counter for readings

// Threshold for change in joystick values to trigger sending data
const int threshold = 25;

bool stateChanged = false;

// Function to update the display with squares corresponding to button states
void updateDisplay(int buttonStates[]) 
{
  for (int i = 0; i < numButtons; i++) 
  {
    if (buttonStates[i] != prevButtonStates[i]) 
    {
      // Only update display if button state has changed
      int color = buttonStates[i] ? ST7735_WHITE : ST7735_BLACK;
      // Choose your square positions and size according to your display and preferences
      int x = i * 20; // Example x position
      int y = 50; // Example y position
      int w = 18; // Example width of square
      int h = 18; // Example height of square
      
      // Update the specific square section for each button
      tft.fillRect(x, y, w, h, color);

      prevButtonStates[i] = buttonStates[i];
    }
  }
}


String message = "";

void readButtonData() 
{
  for (int i = 0; i < numButtons; i++) 
  {
    buttonStates[i] = !digitalRead(buttonPins[i]);
  }
}

int averageX = 0;
int averageY = 0;

void readJoystickData() 
{
  if (readingsCount < numReadings) 
  {
    joystickReadingsX[currentIndex] = analogRead(joystickXPin);
    joystickReadingsY[currentIndex] = analogRead(joystickYPin);
    currentIndex = (currentIndex + 1) % numReadings;
    readingsCount++;
  } 
  else 
  {
    //find the avarage reading
    int totalX = 0;
    int totalY = 0;

    for (int i = 0; i < numReadings; i++) 
    {
      totalX += joystickReadingsX[i];
      totalY += joystickReadingsY[i];
      joystickReadingsX[i] = 0; // Clear the array for next readings
      joystickReadingsY[i] = 0; // Clear the array for next readings
    }

    averageX = totalX / numReadings;
    averageY = totalY / numReadings;

    // Reset variables for next set of readings
    readingsCount = 0;
    currentIndex = 0;
  }
}

void compileButtonStatesSerialData() 
{
  for (int i = 0; i < numButtons; i++) 
  {
    // Check if button state has changed
    if (prevButtonStates[i] != buttonStates[i]) 
    {
      message += "|B" + String(buttonPins[i]) + ":" + String(buttonStates[i]);

      stateChanged = true;
    }
  }
}

void compileJoystickSerialData() 
{
  // Only send the data if the change in joystick value exceeds the threshold
  if (abs(averageX - lastJoystickX) > threshold || abs(averageY - lastJoystickY) > threshold) 
  {
    stateChanged = true;

    lastJoystickX = averageX;
    lastJoystickY = averageY;

    // Append joystick values to the message
    message += "|BX:" + String(lastJoystickX) + "|BY:" + String(lastJoystickY);
  }
}

// Function to handle the main logic of checking button states and sending data
void compileSerialData() 
{
  stateChanged = false;
  message = "START";
  
  compileButtonStatesSerialData();
  compileJoystickSerialData();
  
   message += "|END";
}

void sendSerialData() 
{
  if (stateChanged) 
  {
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
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  // Initialize the ST7735 display
  tft.initR(INITR_144GREENTAB); // Init display with black tab
  tft.fillScreen(ST77XX_BLACK); // Clear the screen

  SerialPort.print("START|END");
}

void loop() 
{
  readButtonData();
  readJoystickData();
  compileSerialData();
  sendSerialData();

  // Update the display
  if(stateChanged)
    updateDisplay(buttonStates);
  
  delay(25); // Delay to avoid excessive looping
}



