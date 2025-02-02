#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <SPI.h>
#include <ESP32Servo.h>
#include <DigiPotX9Cxxx.h>
#include "MPU9250.h"
#include "WiFiPrinter.h"

#define DEBUG_MODE

//ADS1115 joysticks
Adafruit_ADS1115 ads_joystick;
bool is_initialized_ADS1115_joystick = false;

//ADS1115 current and voltage reader
Adafruit_ADS1115 ads_currentSensor;
bool is_initialized_ADS1115_currentSensor = false;

//servos
Servo servoBrake1; 
Servo servoBrake2; 
Servo servoSteering; 

// Define the digital potentiometer with specified multiplexer channels
DigiPot potentiometer1(DigiPot_NC, DigiPot_UD , DigiPot_CS1);  
DigiPot potentiometer2(DigiPot_NC, DigiPot_UD , DigiPot_CS2);

//MPU
MPU9250 IMU(Wire,0x68); 
bool is_initialized_MPU = false;


//MPC23S17 port expander

// Array to map command indices to pin numbers
const uint8_t mosfetPins[] = {MOSFET_1_PIN, MOSFET_2_PIN, MOSFET_3_PIN, MOSFET_4_PIN, MOSFET_5_PIN};
// Array to store the current states of the MOSFETs
bool mosfetStates[] = {LOW, LOW, LOW, LOW, LOW};

// Variables to store previous states of the buttons
bool previousButtonState[4] = {HIGH, HIGH, HIGH, HIGH};

// Variables to store previous states of the sensors
bool previousSensorState[3] = {LOW, LOW, LOW};

bool is_initialized_port_expander = false;

// Declare a variable to store the previous state of the car key switch
bool previousCarKeySwitchState = HIGH;



// Function to set voltmeter voltage using PWM
void setVoltmeterPWM(int pin, int pwmValue, int channel, int freq = 5000, int resolution = 8) 
{
  // Ensure the PWM value is within the valid range
  if (pwmValue < 0) pwmValue = 0;
  if (pwmValue > 255) pwmValue = 255; // Maximum PWM value for 8-bit resolution

  // Attach the pin to a channel with frequency and resolution
  //ledcAttach(pin, freq, resolution);
  ledcAttachChannel(pin, freq, resolution, channel);

  // Write the PWM value directly to the pin
  ledcWrite(pin, pwmValue);
}


void setup()
{
  initializePins();
  neopixelWrite(RGB_BUILTIN,  0,  0,  64);
  initializeSerial();

  initializeServos();
  initializePotentiometers();
}

void loop()
{
  handleCarKeySwitch();
  updateJoysticks();
  updateMPU();
  updateCurrentSensor();
  updatePortExpander();
  updateOverHTTP();

  updateVoltmeters();
  updateServos();
  updatePotentiometers();
}


int joystick_throttle = 0;
int joystick_steering = 0;
int joystick_knob = 0;
float voltage = 0.0;
float current = 0.0;
float inclination_angle = 0.0;


int potValue1 = 0;
int potValue2 = 0;

int servoSteeringValue = 0;

void updateOverHTTP()
{
  static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
  unsigned long currentTime = millis();

  // Check if enough time has been passed since last print call
  if (currentTime - lastUpdateTime >= UPDATE_OVER_HTTP_FREQUENCY) 
  {
      lastUpdateTime = currentTime; // Update the last update time
      WiFiPrinter::printAll(previousCarKeySwitchState,
                          previousButtonState[0], previousButtonState[1], previousButtonState[2], previousButtonState[3], 
                          previousSensorState[1], previousSensorState[2],
                          joystick_throttle, joystick_knob, joystick_steering,
                          voltage,
                          current,
                          inclination_angle);
  }

  WiFiPrinter::update();
}

void initializeSerial()
{
  #ifndef DEBUG_MODE
    return;
  #endif

  Serial.begin(9600);

  // Check if the serial port is available
  unsigned long startMillis = millis();
  while (!Serial && millis() - startMillis < 2000) 
  {
    // Wait up to 2 seconds for the serial connection
    delay(10);
  }
  Serial.println( "Serial startup: " + String ( millis() - startMillis ) );
  

  WiFiPrinter::setup();
  WiFiPrinter::print("Blue Calèche, Bonjour!");
}

void initializeServos() 
{
  //int servoChannel = servoBrake1.attach(SERVO_BRAKE_1);  // Reattach the servo if it was detached
  //Serial.println("servoBrake1 Channel:" + String(servoChannel));

  //servoChannel = servoBrake2.attach(SERVO_BRAKE_2);  // Reattach the servo if it was detached
  //Serial.println("servoBrake2 Channel:" + String(servoChannel));

  int servoChannel = servoSteering.attach(SERVO_STEERING);  // Reattach the servo if it was detached
  //Serial.println("servoSteering Channel:" + String(servoChannel));
  WiFiPrinter::print("servoSteering Channel:" + String(servoChannel));

  delay(50);
}

void initializePotentiometers() 
{
  potentiometer1.set(0);
  potentiometer2.set(0);
}

void initializeMPU() 
{
  if( is_initialized_MPU )
    return;

  unsigned long module_connection_time_Start = millis(); // Record the time when the connection attempt starts

  int status = IMU.begin();

  if (status < 0) 
  {
    WiFiPrinter::print("MPU initialization UNsuccessful!!!!!!!!");
  }
  else
  {
    unsigned long initialization_time = millis() - module_connection_time_Start;
        
    // Print the initialization time
    WiFiPrinter::print("MPU initialized in " + String( initialization_time ) + " ms");

     is_initialized_MPU = true;
  }
}

void initializePins() 
{
  // Initialize pins
  pinMode(CAR_KEY_SWITCH, INPUT_PULLUP);
  pinMode(MOSFET_48V, OUTPUT);
  pinMode(VOLTMETER_SPEED, OUTPUT);
  pinMode(VOLTMETER_CHARGING, OUTPUT);
  pinMode(VOLTMETER_BATTERY, OUTPUT);

  WiFiPrinter::print("Pins initialized");
}







void updateServos()
{
  if(is_initialized_ADS1115_joystick)
  {
    //servoBrake1.write( joystick_throttle );
    //servoBrake2.write( joystick_knob );

    servoSteeringValue = map(joystick_steering, 22, 164, 0, 136);
    servoSteeringValue = constrain(servoSteeringValue, 0, 136); // Ensure it's within 0-180
    servoSteering.write(servoSteeringValue);
  }
}

int joystickMaxValue = 200;
int joystickMiddleValue = 100;
int potMinValue = 30;
int potMaxValue = 63; //75% of the max throttle of the motors. it will translate roughly to 2.85v (.84v the throttle full range is 3.6v)

void updatePotentiometers()
{
  if( !is_initialized_ADS1115_joystick )
    return;

  joystick_knob = constrain(joystick_knob, 0, joystickMaxValue);
  joystick_throttle = constrain(joystick_throttle, 0, joystickMaxValue);


  potValue1 = map(joystick_knob, 0, joystickMaxValue, potMinValue, potMaxValue);
  potValue2 = 0; // Initialize potValue2

  if (joystick_throttle < joystickMiddleValue) 
      potValue2 = 0; // Set potValue2 to 0 if below 100
  else
      // Map joystick_throttle from 100 to 200 to potValue2 from 0 to 75
      potValue2 = map(joystick_throttle, joystickMiddleValue, joystickMaxValue, potMinValue, potMaxValue);







  // Ensure potValue2 never exceeds 75
  potValue1 = min(potValue1, potMaxValue);
  potValue2 = min(potValue2, potMaxValue);

 // Apply the value to the potentiometers
  potentiometer1.set(potValue2);
  potentiometer2.set(potValue1);
}

// Define the necessary variables
unsigned long MPU_LastUpdateTime = 0; // Stores the last update time

void updateMPU()
{
  if( !is_initialized_MPU )
    return;

  unsigned long currentTime = millis();

  // Check if the required time interval has passed
  if (currentTime - MPU_LastUpdateTime < MPU_UPDATE_INTERVAL)
  {
    return; // Exit the method if the interval hasn't passed
  }

  // Update the last update time
  MPU_LastUpdateTime = currentTime;

  // Read sensor data
  IMU.readSensor();

  float accelX = IMU.getAccelX_mss();  // Read X-axis acceleration
  float accelZ = IMU.getAccelZ_mss();  // Read Z-axis acceleration

  // Calculate the angle in degrees
  inclination_angle = atan2(accelX, accelZ) * 180.0 / PI;

  // Print the angle with 1 decimal place
  //Serial.println(inclination_angle, 1);  
}


// Define the necessary variables
unsigned long CurrentSensor_LastUpdateTime = 0; // Stores the last update time

void updateCurrentSensor()
{
  if( !is_initialized_ADS1115_currentSensor )
    return;

  unsigned long currentTime = millis();

  // Check if the required time interval has passed
  if (currentTime - CurrentSensor_LastUpdateTime < CURRENTSENSOR_UPDATE_INTERVAL)
  {
    return; // Exit the method if the interval hasn't passed
  }

  // Update the last update time
  CurrentSensor_LastUpdateTime = currentTime;

  voltage = readBatteryVoltage();
  current = readCurrentSensor(2);
}

float readBatteryVoltage() 
{
    int16_t adcReading = readCurrentSensor(0); // Get ADC reading


    // Calculate and print battery voltage

    float resolutionPerBit = 0.000125; // 0.125 mV/bit for ADS1115 with GAIN_ONE
    float slope = 0.01604;
    float dividerFactor = 0.0642; // Calculate the divider factor
    return adcReading * slope / dividerFactor;// * (resolutionPerBit / dividerFactor); // Calculate battery voltage
}

unsigned long voltMetersLastUpdateTime = 0;  // Variable to store the last update time
void updateVoltmeters()
{
  if( !is_initialized_ADS1115_joystick )
    return;

  unsigned long currentTime = millis();  // Get the current time

  if (currentTime - voltMetersLastUpdateTime >= MPU_UPDATE_INTERVAL) 
  {  // Check if enough time has passed
    setVoltmeterPWM(VOLTMETER_SPEED,    joystick_steering,  VOLTMETER_SPEED_CHANNEL);
    setVoltmeterPWM(VOLTMETER_CHARGING,  joystick_knob,  VOLTMETER_BATTERY_CHANNEL);

    // Define the min and max voltage of the lithium battery pack
    const float minVoltage = 42.0;   // Cutoff voltage (0% charge)
    const float maxVoltage = 54.6;  // Fully charged voltage (100% charge)
    // Constrain voltage to the valid range
    voltage = constrain(voltage, minVoltage, maxVoltage);
    // Map voltage to percentage (0-100)
    int batteryPercentage = map(voltage * 100, minVoltage * 100, maxVoltage * 100, 0, 100);

    // Map percentage to PWM range (0-255)
    int pwmVoltagePercentageValue = map(batteryPercentage, 0, 100, 0, 255);

    setVoltmeterPWM(VOLTMETER_BATTERY, pwmVoltagePercentageValue,  VOLTMETER_CHARGING_CHANNEL);
    
    voltMetersLastUpdateTime = currentTime;  // Update the last update time
  }

}

unsigned long joystickLastUpdateTime = 0;  // Variable to store the last update time

void updateJoysticks()
{
  if( !is_initialized_ADS1115_joystick )
    return;


  unsigned long currentTime = millis();  // Get the current time

  if (currentTime - joystickLastUpdateTime >= MPU_UPDATE_INTERVAL) 
  {  // Check if enough time has passed
    joystick_steering = readJoystick(3);
    joystick_throttle = readJoystick(0);
    joystick_knob = readJoystick(1);
    
    joystickLastUpdateTime = currentTime;  // Update the last update time
  }
}
  
// Function to handle carKeySwitch 
void handleCarKeySwitch()
{
  

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
  WiFiPrinter::print("Turning Car On");

  digitalWrite(MOSFET_48V, HIGH); 
 
  unsigned long module_connection_time_Start = millis(); // Record the time when the connection attempt starts

  while (millis() - module_connection_time_Start < MODULE_CONNECTION_TIMEOUT)
  {
    if (ads_joystick.begin(0x4A)) 
    {
        is_initialized_ADS1115_joystick = true;
        ads_joystick.setGain(GAIN_ONE);

        // Calculate how long it took to initialize in ms
        unsigned long initialization_time = millis() - module_connection_time_Start;

        // Print the initialization time
        WiFiPrinter::print("ADS1115 joystick initialized in " + String( initialization_time ) + "ms");

        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }

  module_connection_time_Start = millis();
  while (millis() - module_connection_time_Start < MODULE_CONNECTION_TIMEOUT)
  {
    if (ads_currentSensor.begin()) 
    {
        is_initialized_ADS1115_currentSensor = true;
        ads_currentSensor.setGain(GAIN_ONE);

        unsigned long initialization_time = millis() - module_connection_time_Start;
        
        // Print the initialization time
        WiFiPrinter::print("ADS1115 currentSensor initialized in " + String( initialization_time ) + "ms");

        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }


  
  initializeMPU();


  //MCP23S17
  // Initialize SPI Communication for display and port expander
  module_connection_time_Start = millis();

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
  pinModeMCP23S17('A', SENSOR_5V_EMPTY_PIN, INPUT);

  // Initialize MOSFET pins as outputs
  for (uint8_t i = 0; i < 5; i++) 
  {
    pinModeMCP23S17('A', mosfetPins[i], OUTPUT);
    digitalWriteMCP23S17('A', mosfetPins[i], LOW); // Default to LOW
  }


  //set empty pins as output low
  pinModeMCP23S17('B', 4, OUTPUT);
  digitalWriteMCP23S17('B', 4, LOW); // Default to LOW
  pinModeMCP23S17('B', 5, OUTPUT);
  digitalWriteMCP23S17('B', 5, LOW); // Default to LOW
  pinModeMCP23S17('B', 6, OUTPUT);
  digitalWriteMCP23S17('B', 6, LOW); // Default to LOW
  pinModeMCP23S17('B', 7, OUTPUT);
  digitalWriteMCP23S17('B', 7, LOW); // Default to LOW

  // Check if the module is initialized (simple check), Check IODIR register
  if (readMCP23S17(0x00) == 0x00) 
  {  
    WiFiPrinter::print("MCP23S17 Initialization Failed!");

  } 
  else 
  {
    unsigned long initialization_time = millis() - module_connection_time_Start;
        
    // Print the initialization time
    WiFiPrinter::print("MCP23S17 initialized in " + String( initialization_time ) + "ms");

    is_initialized_port_expander = true;
  }
  




  // If initialization failed after the timeout, call the error handling method
  if (!is_initialized_ADS1115_joystick) 
  {
    registerError( "Could not inilialize ADS1115 for joystick" );
  }

  // If initialization failed after the timeout, call the error handling method
  if (!is_initialized_ADS1115_currentSensor) 
  {
    registerError( "Could not inilialize ADS1115 for currentSensor" );
  }


  neopixelWrite(RGB_BUILTIN,0,24,0); // Green

  WiFiPrinter::print("Car Turned On");
}



void turnOffCar()
{
  WiFiPrinter::print("Turning Car Off");

  potentiometer1.set(0);
  potentiometer2.set(0);

  if( is_initialized_ADS1115_joystick)
  {
    setVoltmeterPWM(VOLTMETER_SPEED,    0,  VOLTMETER_SPEED_CHANNEL   );
    setVoltmeterPWM(VOLTMETER_CHARGING, 0,  VOLTMETER_CHARGING_CHANNEL);
    setVoltmeterPWM(VOLTMETER_BATTERY,  0,  VOLTMETER_BATTERY_CHANNEL );
  }
  

  neopixelWrite(RGB_BUILTIN,  64,  0,  0); // redish
  digitalWrite( MOSFET_48V, LOW); 

  is_initialized_ADS1115_joystick = false;
  is_initialized_ADS1115_currentSensor = false;
  is_initialized_port_expander = false;
  is_initialized_MPU = false;

  WiFiPrinter::print("Car Turned Off");
}

// Function to read joystick value from ADS1115 A2 pin and map to PWM
int readJoystick( int adcPin ) 
{
  // Read ADC value (16-bit signed integer, -32768 to 32767)
  int16_t rawValue = ads_joystick.readADC_SingleEnded( adcPin );

  // Convert raw ADC value to PWM range (0-255)
  // ADS1115 range: 0-32767 maps to 0-3.3V
  int pwmValue = map(rawValue, 0, 32767, 0, PWM_RESOLUTION);

  // Ensure PWM value is within the valid range
  pwmValue = constrain(pwmValue, 0, PWM_RESOLUTION);

  return pwmValue;
}

// Function to read joystick value from ADS1115 A2 pin and map to PWM
int readCurrentSensor( int adcPin ) 
{
  // Read ADC value (16-bit signed integer, -32768 to 32767)
  int16_t rawValue = ads_currentSensor.readADC_SingleEnded( adcPin );

  // Convert raw ADC value to PWM range (0-255)
  // ADS1115 range: 0-32767 maps to 0-3.3V
  int pwmValue = map(rawValue, 0, 32767, 0, PWM_RESOLUTION);

  // Ensure PWM value is within the valid range
  pwmValue = constrain(pwmValue, 0, PWM_RESOLUTION);

  return pwmValue;
}

void registerError(const char* errorMessage)
{
    WiFiPrinter::print(errorMessage);
}



void updatePortExpander() 
{
  if( !is_initialized_port_expander)
    return;

  updateButtons();
  updateSensors();
}

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;  // milliseconds
void updateButtons()
{
  if (millis() - lastDebounceTime > debounceDelay) 
  {
    for (uint8_t i = 0; i < 4; i++)
    {
      uint8_t currentButtonState = digitalReadMCP23S17('B', i);  // Read current state of button

      // If the button state has changed, print the new state
      if (currentButtonState != previousButtonState[i]) 
      {
        if (currentButtonState == HIGH) 
        {
          Serial.println("Button " + String( i ) + " HIGH");
          previousButtonState[i] = HIGH;
          digitalWriteMCP23S17('A', mosfetPins[i], LOW);
        }
        else 
        {
          Serial.println("Button " + String( i ) + " LOW");
          previousButtonState[i] = LOW;
          digitalWriteMCP23S17('A', mosfetPins[i], HIGH);
        }

      }
    }

    lastDebounceTime = millis();
  }
}

void updateSensors()
{
  uint8_t currentSensorState = digitalReadMCP23S17('A', SENSOR_WHEEL_SPEED_PIN);  // Read current state of sensor
  if (currentSensorState != previousSensorState[1]) 
    previousSensorState[1] = currentSensorState;
  
  currentSensorState = digitalReadMCP23S17('A', SENSOR_PEDAL_TRIGGER_PIN);  // Read current state of sensor
  if (currentSensorState != previousSensorState[2]) 
    previousSensorState[2] = currentSensorState;
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