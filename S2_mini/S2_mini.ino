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
const int numJoystickReadings = 50; // Number of readings to smooth joystick input
int joystickReadingsX[numJoystickReadings] = {0}; // Array to store joystick X readings
int joystickReadingsY[numJoystickReadings] = {0}; // Array to store joystick Y readings
unsigned int currentJoystickReadingtIndex = 0; // Index for readings arrays
unsigned int readingsCount = 0; // Counter for readings
int averageJoystickX = 0;
int averageJoystickY = 0;
const int threshold = 25; // Threshold for change in joystick values to trigger sending data




bool stateChanged = false;

String UARTMessage = "";



// battery voltage reading
const int INTERVAL_BETWEEN_VOLTAGE_READINGS_CYCLE = 5000;
const int MAX_VOLTAGE_READINGS_PER_CYCLE = 100;
const int batteryVoltagePin = 11;
int avarageBatteryADCReading  = 0;
int voltageReadingsCount = 0; // Counter for readings
unsigned long previousVoltageTime = 0;

int voltageReadings[MAX_VOLTAGE_READINGS_PER_CYCLE];
bool isNewVoltageADCReadingAvailable = false;

void drawText(int x, int y, String text, int color) 
{
  tft.setCursor(x, y);
  tft.setTextColor(color);
  tft.print(text);
}

void clearLine(int line) {
  tft.fillRect(0, line * 8, 128, 8, ST7735_WHITE); // Adjust the screen width accordingly
}

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
      int y = 0; // Example y position
      int w = 18; // Example width of square
      int h = 18; // Example height of square
      
      // Update the specific square section for each button
      tft.fillRect(x, y, w, h, color);

      prevButtonStates[i] = buttonStates[i];
    }
  }

  updateJoystickDisplay();


  if( isNewVoltageADCReadingAvailable )
  {
    clearLine(1); // Clear line 0
    drawText(0, 8, " ADC: " + String(avarageBatteryADCReading), ST7735_BLACK);

    clearLine(2); // Clear line 1
    drawText(0, 16, " " + String(calculateBatteryVoltage(avarageBatteryADCReading)) + "v", ST7735_BLACK);

    clearLine(3); // Clear line 2
    drawText(0, 24, " " + String(calculateBatteryPercentage(avarageBatteryADCReading)) + "%", ST7735_BLACK);
  }

  isNewVoltageADCReadingAvailable = false;

}

int pixel_x = 0;
int pixel_y = 0;

int square_x = 0; //  x position
int square_y = 0; //  y position
int square_width = 128; //  width of square
int joystick_maxValue = 2048;

void updateJoystickDisplay() 
{
  tft.drawPixel(pixel_x, pixel_y, ST7735_WHITE);
  tft.drawPixel(pixel_x+1, pixel_y, ST7735_WHITE);
  tft.drawPixel(pixel_x, pixel_y+1, ST7735_WHITE);
  tft.drawPixel(pixel_x-1, pixel_y, ST7735_WHITE);
  tft.drawPixel(pixel_x, pixel_y-1, ST7735_WHITE);

  // Map joystick values to screen coordinates
  pixel_x =  map(lastJoystickX, joystick_maxValue, 0, square_x, square_x + square_width); // Map joystick X value to screen width
  pixel_y =  map(lastJoystickY, 0, joystick_maxValue, square_y, square_y + square_width); // Map joystick Y value to screen height

  tft.drawPixel(pixel_x, pixel_y, ST7735_BLACK);
  tft.drawPixel(pixel_x+1, pixel_y, ST7735_BLACK);
  tft.drawPixel(pixel_x, pixel_y+1, ST7735_BLACK);
  tft.drawPixel(pixel_x-1, pixel_y, ST7735_BLACK);
  tft.drawPixel(pixel_x, pixel_y-1, ST7735_BLACK);
  

  // Draw the frame for the joystick display area
  //tft.drawRect(square_x, square_y, square_width, square_width, ST7735_BLACK);
}



void readButtonData() 
{
  for (int i = 0; i < numButtons; i++) 
  {
    buttonStates[i] = !digitalRead(buttonPins[i]);
  }
}




float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) 
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float calculateBatteryVoltage(int adcReading) 
{
  float batteryVoltage = mapFloat(adcReading, 627, 822, 3.3, 4.0);
  return batteryVoltage;
}

int calculateBatteryPercentage(int adcReading) 
{
  float percentage = cubicEaseInOut(mapFloat(adcReading, 625, 825, 0.0, 1.0));
  return int(percentage * 100);
}

// Cubic easing function
float cubicEaseInOut(float t) 
{
  return t < 0.5 ? 4 * t * t * t : 1 - pow(-2 * t + 2, 3) / 2;
}

void readVoltageData() 
{
  // Check if enough time has passed since the last voltage interval
  unsigned long currentMillis = millis();

  if (currentMillis - previousVoltageTime >= INTERVAL_BETWEEN_VOLTAGE_READINGS_CYCLE) 
  {
    // Check if the number of readings is less than the maximum
    if (voltageReadingsCount < MAX_VOLTAGE_READINGS_PER_CYCLE) 
    {
      // Read voltage and store in the array
      voltageReadings[voltageReadingsCount] = analogRead(batteryVoltagePin);
      voltageReadingsCount++;

      // If reached the maximum readings, compile and send data
      if (voltageReadingsCount >= MAX_VOLTAGE_READINGS_PER_CYCLE) 
      {
        long voltageSum = 0;

        for (int i = 0; i < MAX_VOLTAGE_READINGS_PER_CYCLE; i++) 
        {
            voltageSum += voltageReadings[i];
        }

        avarageBatteryADCReading = voltageSum / MAX_VOLTAGE_READINGS_PER_CYCLE;

        // Reset readings count and time counter
        voltageReadingsCount = 0;
        previousVoltageTime = currentMillis;
        isNewVoltageADCReadingAvailable = true;
      }
    }
  }
}

void compileVoltageSerialData() 
{
  if( isNewVoltageADCReadingAvailable )
  {
    UARTMessage += "|Br:" + String( avarageBatteryADCReading );
    UARTMessage += "|Bv:" + String( calculateBatteryVoltage(avarageBatteryADCReading) );
    UARTMessage += "|B%:" + String( calculateBatteryPercentage(avarageBatteryADCReading) );


    stateChanged = true;
    //isNewVoltageADCReadingAvailable = false;
  }
}




void readJoystickData() 
{
  if (readingsCount < numJoystickReadings) 
  {
    joystickReadingsX[currentJoystickReadingtIndex] = analogRead(joystickXPin);
    joystickReadingsY[currentJoystickReadingtIndex] = analogRead(joystickYPin);
    currentJoystickReadingtIndex = (currentJoystickReadingtIndex + 1) % numJoystickReadings;
    readingsCount++;
  } 
  else 
  {
    //find the avarage reading
    int totalX = 0;
    int totalY = 0;

    for (int i = 0; i < numJoystickReadings; i++) 
    {
      totalX += joystickReadingsX[i];
      totalY += joystickReadingsY[i];
      joystickReadingsX[i] = 0; // Clear the array for next readings
      joystickReadingsY[i] = 0; // Clear the array for next readings
    }

    averageJoystickX = totalX / numJoystickReadings;
    averageJoystickY = totalY / numJoystickReadings;

    // Reset variables for next set of readings
    readingsCount = 0;
    currentJoystickReadingtIndex = 0;
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
    // Delay to avoid excessive looping
    delay(5); 
  }
}

void setup() 
{
  Serial.begin(115200);
  SerialPort.begin(115200, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);


  // Configure ADC for ESP32-S2
  analogReadResolution(11); // Set ADC resolution to 12 bits

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


