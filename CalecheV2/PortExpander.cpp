#include "esp32-hal-gpio.h"
#include "PortExpander.h"
#include "PinDefinitions.h"
#include "WiFiPrinter.h"
#include <SPI.h>
#include <Arduino.h>

// Initialize the MCP23S17
void PortExpander::start() 
{
  unsigned long module_connection_time_Start = millis();

  // Initialize SPI Communication cs pin
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);  // Keep CS pin high initially

  // Set GPIO B0 to B3 as input
  pinModeMCP23S17('B', BUTTON_0_PIN, INPUT);
  pinModeMCP23S17('B', BUTTON_1_PIN, INPUT);
  pinModeMCP23S17('B', BUTTON_2_PIN, INPUT);
  pinModeMCP23S17('B', BUTTON_3_PIN, INPUT);
  pinModeMCP23S17('A', SENSOR_PEDAL_TRIGGER_PIN, INPUT);
  pinModeMCP23S17('A', SENSOR_WHEEL_SPEED_PIN, INPUT);

  //mosfets
  pinModeMCP23S17('A', MOSFET_BRAKE_LIGHT_PIN, OUTPUT);
  pinModeMCP23S17('A', MOSFET_HORN_PIN, OUTPUT);
  pinModeMCP23S17('A', MOSFET_NIGH_LIGHT_PIN, OUTPUT);
  pinModeMCP23S17('A', MOSFET_BRAKE_PIN, OUTPUT);
  pinModeMCP23S17('A', MOSFET_REVERSE_PIN, OUTPUT);

  digitalWriteMCP23S17('A', MOSFET_BRAKE_LIGHT_PIN, LOW);
  digitalWriteMCP23S17('A', MOSFET_HORN_PIN, LOW);
  digitalWriteMCP23S17('A', MOSFET_NIGH_LIGHT_PIN, LOW);
  digitalWriteMCP23S17('A', MOSFET_BRAKE_PIN, LOW);
  digitalWriteMCP23S17('A', MOSFET_REVERSE_PIN, LOW);


  pinModeMCP23S17('B', 4, OUTPUT);
  pinModeMCP23S17('B', 5, INPUT_PULLUP);
  pinModeMCP23S17('B', 6, INPUT_PULLUP);
  pinModeMCP23S17('B', 7, INPUT_PULLUP);
  pinModeMCP23S17('A', SENSOR_5V_EMPTY_PIN, INPUT_PULLUP);



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

    initialized = true;
  } 
}

// Deinitialize the MCP23S17
void PortExpander::shutdown()
{
    if (initialized) 
    {
        initialized = false;
    }
}

// Update function (implement as needed)
void PortExpander::update() 
{
   
}

// Public digitalWrite method
void PortExpander::digitalWrite(uint8_t pin, uint8_t value) 
{
    digitalWriteMCP23S17('A', pin, value);
}

// Public digitalRead method
uint8_t PortExpander::digitalRead(uint8_t pin) 
{
  uint8_t port ='B';

  if( pin == SENSOR_PEDAL_TRIGGER_PIN || pin == SENSOR_WHEEL_SPEED_PIN)
    port = 'A';

  return digitalReadMCP23S17(port, pin);
}

// Private methods for MCP23S17 operations
void PortExpander::writeMCP23S17(uint8_t registerAddress, uint8_t data) {
    ::digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x40); // Write command
    SPI.transfer(registerAddress);
    SPI.transfer(data);
    ::digitalWrite(CS_PIN, HIGH);
}

uint8_t PortExpander::readMCP23S17(uint8_t registerAddress) 
{
    ::digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x41); // Read command
    SPI.transfer(registerAddress);
    uint8_t data = SPI.transfer(0x00);
    ::digitalWrite(CS_PIN, HIGH);
    return data;
}

void PortExpander::pinModeMCP23S17(uint8_t port, uint8_t pin, uint8_t mode) {
    uint8_t registerAddress = (port == 'A') ? 0x00 : 0x01; // IODIRA or IODIRB
    uint8_t currentIODIR = readMCP23S17(registerAddress);
    if (mode == INPUT) {
        currentIODIR |= (1 << pin);
    } else {
        currentIODIR &= ~(1 << pin);
    }
    writeMCP23S17(registerAddress, currentIODIR);
}

void PortExpander::pullUpMCP23S17(uint8_t port, uint8_t pin, bool enable) {
    uint8_t registerAddress = (port == 'A') ? 0x0C : 0x0D; // GPPUA or GPPUB
    uint8_t currentGPPU = readMCP23S17(registerAddress);
    if (enable) {
        currentGPPU |= (1 << pin);
    } else {
        currentGPPU &= ~(1 << pin);
    }
    writeMCP23S17(registerAddress, currentGPPU);
}

uint8_t PortExpander::digitalReadMCP23S17(uint8_t port, uint8_t pin) {
    uint8_t registerAddress = (port == 'A') ? 0x12 : 0x13; // GPIOA or GPIOB
    uint8_t currentGPIO = readMCP23S17(registerAddress);
    return (currentGPIO & (1 << pin)) ? HIGH : LOW;
}

void PortExpander::digitalWriteMCP23S17(uint8_t port, uint8_t pin, uint8_t value) {
    uint8_t registerAddress = (port == 'A') ? 0x12 : 0x13; // GPIOA or GPIOB
    uint8_t currentGPIO = readMCP23S17(registerAddress);
    if (value == HIGH) {
        currentGPIO |= (1 << pin);
    } else {
        currentGPIO &= ~(1 << pin);
    }
    writeMCP23S17(registerAddress, currentGPIO);
}
