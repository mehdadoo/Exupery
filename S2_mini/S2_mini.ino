#include <HardwareSerial.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Arduino.h>
#include <driver/adc.h>

// Use UART1
HardwareSerial SerialPort(1);

// Pins for UART1
const int SERIAL_TX_PIN = 40;
const int SERIAL_RX_PIN = 38;

// ST7735 display pins
#define TFT_SCK 18
#define TFT_SDA 33
#define TFT_A0   35
#define TFT_RESET  37 
#define TFT_CS   39

const int ledPin = 3;
const int ledBluePin = 12;


const int buttonPins[] = {5, 7, 13, 16, 8, 10, 6};

const int joystickYPin = 4; // Joystick Y pin connected to GPIO 4
const int joystickXPin = 2; // Joystick X pin connected to GPIO 6

const int batteryVoltagePin = 11;
int voltageReading = 0;
int prevVoltageReading = 0;


// Initialize Adafruit ST7735 with hardware SPI
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_A0, TFT_SDA, TFT_SCK, TFT_RESET);

// Button pin assignments
const int numButtons = sizeof(buttonPins) / sizeof(buttonPins[0]);

// Variables to track previous button states
int buttonStates[numButtons] = {0};
int prevButtonStates[numButtons] = {0};

// Joystick last known values
int lastJoystickX = -1; // Initialize with invalid value to make sure it sends the first reading
int lastJoystickY = -1; // Initialize with invalid value to make sure it sends the first reading
int smoothedJoystickX = 0; // Smoothed joystick X value
int smoothedJoystickY = 0; // Smoothed joystick Y value
const int numReadings = 10; // Number of readings to smooth joystick input
int joystickReadingsX[numReadings] = {0}; // Array to store joystick X readings
int joystickReadingsY[numReadings] = {0}; // Array to store joystick Y readings
unsigned int currentIndex = 0; // Index for readings arrays
unsigned int readingsCount = 0; // Counter for readings
int averageJoystickX = 0;
int averageJoystickY = 0;
const int threshold = 25; // Threshold for change in joystick values to trigger sending data




bool stateChanged = false;

String UARTMessage = "";



// Function to update the display with squares corresponding to button states
void updateDisplay(int buttonStates[]) 
{
  for (int i = 0; i < numButtons; i++) 
  {
    if (buttonStates[i] != prevButtonStates[i]) 
    {
      // Only update display if button state has changed
      int color = buttonStates[i] ? ST7735_BLACK : ST7735_WHITE;
      // Choose your square positions and size according to your display and preferences
      int x = i * 18; // Example x position
      int y = 50; // Example y position
      int w = 18; // Example width of square
      int h = 18; // Example height of square
      
      // Update the specific square section for each button
      tft.fillRect(x, y, w, h, color);

      prevButtonStates[i] = buttonStates[i];
    }
  }

 // Clear a white rectangle behind the text
  int xTextPos = 3; // X position for text
  int yTextPos = 50 + 18 + 5; // Y position for text, adjust as needed
  int textWidth = 50; // Adjust based on the width of text

  tft.fillRect(xTextPos, yTextPos, textWidth, 30, ST7735_WHITE); // Fill a white rectangle behind the text

  tft.setCursor(xTextPos, yTextPos); // Set cursor position for text
  tft.setTextColor(ST7735_BLACK); // Text color
  tft.setTextSize(1); // Text size
  tft.println("X: " + String(lastJoystickX)); // Display lastJoystickX

  yTextPos += 10; // Move to the next line for Y value
  tft.setCursor(xTextPos, yTextPos); // Set cursor position for text
  tft.println("Y: " + String(lastJoystickY)); // Display lastJoystickY

  // Add the voltage display code here
  
  
}



void readButtonData() 
{
  for (int i = 0; i < numButtons; i++) 
  {
    buttonStates[i] = !digitalRead(buttonPins[i]);
  }
}



void readVoltageData() 
{
  if (readingsCount == 0) 
  {
    voltageReading = analogRead(batteryVoltagePin);

  }
}

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

    averageJoystickX = totalX / numReadings;
    averageJoystickY = totalY / numReadings;

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
      UARTMessage += "|B" + String(i) + ":" + String(buttonStates[i]);

      stateChanged = true;
    }
  }
}

void compileJoystickSerialData() 
{
  // Only send the data if the change in joystick value exceeds the threshold
  if (abs(averageJoystickX - lastJoystickX) > threshold || abs(averageJoystickY - lastJoystickY) > threshold) 
  {
    stateChanged = true;

    lastJoystickX = averageJoystickX;
    lastJoystickY = averageJoystickY;

    // Append joystick values to the UARTMessage
    UARTMessage += "|BX:" + String(lastJoystickX) + "|BY:" + String(lastJoystickY);
  }
}



void compileVoltageSerialData() 
{
  return;
    if( voltageReading != prevVoltageReading)
    {
      float voltage = (float)voltageReading / 4095 * 4200; // Convert reading to voltage
      UARTMessage += "|BR:" + String(voltageReading);
      UARTMessage += "|BV:" + String(voltage);

      voltageReading = prevVoltageReading;
      stateChanged = true;
    }
}

// Function to handle the main logic of checking button states and sending data
void compileSerialData() 
{
  stateChanged = false;
  UARTMessage = "START";
  
  compileButtonStatesSerialData();
  compileJoystickSerialData();
  compileVoltageSerialData();
  
   UARTMessage += "|END";
}

void sendSerialData() 
{
  if (stateChanged) 
  {
    // Print the UARTMessage to Serial Monitor
    Serial.println(UARTMessage);
    // Send the UARTMessage over UART
    SerialPort.print(UARTMessage);
  }
}

void setup() 
{
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);


  // Configure ADC for ESP32-S2
  analogReadResolution(12); // Set ADC resolution to 12 bits

  // Configure ADC attenuation for the battery voltage pin
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11); // Assuming ADC1_CHANNEL_0, adjust if necessary

  pinMode(batteryVoltagePin, INPUT);


  // Initialize button pins as inputs with internal pull-up
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }

  pinMode(ledPin, OUTPUT);
  pinMode(ledBluePin, OUTPUT);


  // Initialize the ST7735 display
  tft.initR(INITR_144GREENTAB); // Init display with black tab
  tft.fillScreen(ST7735_WHITE); // Clear the screen

  SerialPort.print("START|BX:0|BY:0|END");
}

void loop() 
{
  readButtonData();
  readJoystickData();
  readVoltageData();
  compileSerialData();
  sendSerialData();

  // Update the display
  if(stateChanged)
    updateDisplay(buttonStates);

  controlLED();
  
  delay(25); // Delay to avoid excessive looping
}

void controlLED() 
{
  // If button connected to pin 3 is off, turn on the LED, else turn it off
  if(buttonStates[0]) 
  { // Change to the index of the button connected to pin 3
    digitalWrite(ledPin, HIGH); // Turn on LED
  }
  else
  {
    digitalWrite(ledPin, LOW); // Turn off LED
  }

  if(buttonStates[1]) 
  { // Change to the index of the button connected to pin 3
    digitalWrite(ledBluePin, HIGH); // Turn on LED
  }
  else
  {
    digitalWrite(ledBluePin, LOW); // Turn off LED
  }
}


