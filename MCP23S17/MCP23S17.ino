#include <SPI.h>
#include <Arduino.h>

#define MOSFET_48V      34   // Output for 48V MOSFET
#define CAR_KEY_SWITCH  15   // Input for car key switch

// Pin Definitions for SPI Communication
#define SCK_PIN 12   // Clock
#define MOSI_PIN 13  // Master Out Slave In
#define MISO_PIN 37  // Master In Slave Out
#define CS_PIN 38    // Chip Select

// GPIO Pin Definitions for Buttons (GPB0 to GPB3)
#define BUTTON_0_PIN 0   // GPB0 (Pin 0 on Bank B)
#define BUTTON_1_PIN 1   // GPB1 (Pin 1 on Bank B)
#define BUTTON_2_PIN 2   // GPB2 (Pin 2 on Bank B)
#define BUTTON_3_PIN 3   // GPB3 (Pin 3 on Bank B)

// GPIO Pin Definitions for MOSFET Control
#define MOSFET_1_PIN 0  // GPIO A0 (Pin 0 on Bank A)
#define MOSFET_2_PIN 1  // GPIO A1 (Pin 1 on Bank A)
#define MOSFET_3_PIN 2  // GPIO A2 (Pin 2 on Bank A)
#define MOSFET_4_PIN 6  // GPIO A6 (Pin 6 on Bank A)
#define MOSFET_5_PIN 7  // GPIO A7 (Pin 7 on Bank A)

// GPIO Pin Definitions for Sensors
#define SENSOR_PEDAL_TRIGGER_PIN 5  //GPIO A5 (Pin 5 on Bank A)
#define SENSOR_5V_EMPTY_PIN 4    //GPIO A4 (Pin 4 on Bank A)
#define SENSOR_WHEEL_SPEED_PIN 3    //GPIO A3 (Pin 3 on Bank A)

// Array to map command indices to pin numbers
const uint8_t mosfetPins[] = {MOSFET_1_PIN, MOSFET_2_PIN, MOSFET_3_PIN, MOSFET_4_PIN, MOSFET_5_PIN};

// Array to store the current states of the MOSFETs
bool mosfetStates[] = {LOW, LOW, LOW, LOW, LOW};


void setup() {
  // Initialize Serial Monitor
  Serial.begin(9600);

  // Initialize pins
  pinMode(CAR_KEY_SWITCH, INPUT_PULLUP);
  pinMode(MOSFET_48V, OUTPUT);

  
}

// Variables to store previous states of the buttons
uint8_t previousButtonState[4] = {HIGH, HIGH, HIGH, HIGH};

// Variables to store previous states of the sensors
uint8_t previousSensorState[3] = {LOW, LOW, LOW};

bool initilized = false;


 
// Function to handle carKeySwitch 
void handleCarKeySwitch()
{
  // Declare a static variable to store the previous state of the car key switch
  static int previousCarKeySwitchState = HIGH;

  // Read the current state of the car key switch
  int carKeySwitchState = digitalRead(CAR_KEY_SWITCH);


  // Check if the state has changed
  if (carKeySwitchState != previousCarKeySwitchState) 
  {
    if (carKeySwitchState == LOW) 
    {
      turnOnCar();
    }
    else 
    {
      turnOffCar();
    }

    // Update the previous state
    previousCarKeySwitchState = carKeySwitchState;
  }
}

void turnOnCar()
{
  digitalWrite(MOSFET_48V, HIGH); 

  delay(250);

  // Initialize SPI Communication
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);  // Keep CS pin high initially

  // Set GPIO B0 to B3 as input with pull-up enabled
  pinModeMCP23S17('B', BUTTON_0_PIN, INPUT);
  pinModeMCP23S17('B', BUTTON_1_PIN, INPUT);
  pinModeMCP23S17('B', BUTTON_2_PIN, INPUT);
  pinModeMCP23S17('B', BUTTON_3_PIN, INPUT);
  pinModeMCP23S17('A', SENSOR_PEDAL_TRIGGER_PIN, INPUT);
  pinModeMCP23S17('A', SENSOR_WHEEL_SPEED_PIN, INPUT);



  // Initialize MOSFET pins as outputs
  for (uint8_t i = 0; i < 5; i++) 
  {
    pinModeMCP23S17('A', mosfetPins[i], OUTPUT);
    digitalWriteMCP23S17('A', mosfetPins[i], LOW); // Default to LOW
  }

  Serial.println( readMCP23S17(0x00) );

  // Check if the module is initialized (simple check)
  if (readMCP23S17(0x00) == 0x00) {  // Check IODIR register
    Serial.println("MCP23S17 Initialization Failed!");
  } else {
    Serial.println("MCP23S17 Initialized Successfully!");
    initilized = true;
  }




  neopixelWrite(RGB_BUILTIN,0,24,0); // Green

  Serial.println("Car Turned On");
}

void turnOffCar()
{
  neopixelWrite(RGB_BUILTIN,  64,  0,  0); // redish
  digitalWrite( MOSFET_48V, LOW); 
  Serial.println("Car Turned Off");
  initilized = false;
}



void loop() {

  handleCarKeySwitch();

  if(!initilized)
    return;

  // Check the state of each button (GPB0 to GPB3)
  for (uint8_t i = 0; i < 4; i++) {
    uint8_t currentButtonState = digitalReadMCP23S17('B', i);  // Read current state of button

    // If the button state has changed, print the new state
    if (currentButtonState != previousButtonState[i]) {
      if (currentButtonState == HIGH) {
        Serial.print("Button ");
        Serial.print(i);
        Serial.println(" HIGH");
        digitalWriteMCP23S17('A', mosfetPins[i], LOW);
      } else {
        Serial.print("Button ");
        Serial.print(i);
        Serial.println(" low");
        digitalWriteMCP23S17('A', mosfetPins[i], HIGH);
      }
      // Update the previous state
      previousButtonState[i] = currentButtonState;
    }
  }

  
  uint8_t currentSensorState = digitalReadMCP23S17('A', SENSOR_WHEEL_SPEED_PIN);  // Read current state of sensor

  // If the button state has changed, print the new state
  if (currentSensorState != previousSensorState[1]) 
  {
    if (currentSensorState == HIGH) 
    {
      Serial.print("SENSOR_WHEEL_SPEED: ");
      Serial.println(" HIGH");
    } 
    else 
    {
      Serial.print("SENSOR_WHEEL_SPEED: ");
      Serial.println(" LOW");
    }
    // Update the previous state
    previousSensorState[1] = currentSensorState;
  }
  /*
  currentSensorState = digitalReadMCP23S17('A', SENSOR_PEDAL_TRIGGER_PIN);  // Read current state of sensor

  // If the button state has changed, print the new state
  if (currentSensorState != previousSensorState[2]) 
  {
    if (currentSensorState == HIGH) 
    {
      //Serial.print("SENSOR_WHEEL_SPEED: ");
      Serial.println(r++);
    } 
    else 
    {
      //Serial.print("SENSOR_WHEEL_SPEED: ");
      //Serial.println(" LOW");
    }
    // Update the previous state
    previousSensorState[2] = currentSensorState;
  }
  */
  

  delay(50);  // Polling delay
}



// Function to write to MCP23S17
void writeMCP23S17(uint8_t registerAddress, uint8_t data) {
  digitalWrite(CS_PIN, LOW);  // Select the MCP23S17
  SPI.transfer(0x40);          // Write command (0x40 for MCP23S17)
  SPI.transfer(registerAddress); // Register address
  SPI.transfer(data);          // Data to write
  digitalWrite(CS_PIN, HIGH);  // Deselect the MCP23S17
}

// Function to read from MCP23S17
uint8_t readMCP23S17(uint8_t registerAddress) {
  digitalWrite(CS_PIN, LOW);  // Select the MCP23S17
  SPI.transfer(0x41);          // Read command (0x41 for MCP23S17)
  SPI.transfer(registerAddress); // Register address
  uint8_t data = SPI.transfer(0x00); // Read data
  digitalWrite(CS_PIN, HIGH);  // Deselect the MCP23S17
  return data;
}

// Function to configure pin mode on MCP23S17
void pinModeMCP23S17(uint8_t port, uint8_t pin, uint8_t mode) {
  uint8_t registerAddress = (port == 'A') ? 0x00 : 0x01;  // IODIRA or IODIRB
  uint8_t currentIODIR = readMCP23S17(registerAddress);   // Read the current IODIR register for the specified port

  if (mode == INPUT) {
    currentIODIR |= (1 << pin);  // Set pin as input (1 = input)
  } else {
    currentIODIR &= ~(1 << pin); // Set pin as output (0 = output)
  }

  writeMCP23S17(registerAddress, currentIODIR);          // Write back to the IODIR register
}


// Function to enable pull-up resistors on MCP23S17
void pullUpMCP23S17(uint8_t port, uint8_t pin, bool enable) {
  uint8_t registerAddress = (port == 'A') ? 0x0C : 0x0D;  // GPPUA or GPPUB
  uint8_t currentGPPU = readMCP23S17(registerAddress);    // Read the current GPPU register for the specified port

  if (enable) {
    currentGPPU |= (1 << pin);  // Enable pull-up resistor
  } else {
    currentGPPU &= ~(1 << pin); // Disable pull-up resistor
  }

  writeMCP23S17(registerAddress, currentGPPU);           // Write back to the GPPU register
}


// Function to read a pin on MCP23S17 (digital read)
uint8_t digitalReadMCP23S17(uint8_t port, uint8_t pin) {
  uint8_t registerAddress = (port == 'A') ? 0x12 : 0x13;  // GPIOA or GPIOB
  uint8_t currentGPIO = readMCP23S17(registerAddress);    // Read the current GPIO register for the specified port
  return (currentGPIO & (1 << pin)) ? HIGH : LOW;         // Return HIGH or LOW based on the pin state
}


void digitalWriteMCP23S17(uint8_t port, uint8_t pin, uint8_t value) {
  uint8_t registerAddress = (port == 'A') ? 0x12 : 0x13;  // GPIOA or GPIOB
  uint8_t currentGPIO = readMCP23S17(registerAddress);    // Read the current GPIO register for the specified port
  if (value == HIGH) {
    currentGPIO |= (1 << pin);  // Set the bit to HIGH
  } else {
    currentGPIO &= ~(1 << pin); // Clear the bit to LOW
  }
  writeMCP23S17(registerAddress, currentGPIO);           // Write back to the GPIO register
}