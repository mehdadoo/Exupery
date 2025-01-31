#include "esp32-hal-gpio.h"
#include "PortExpander.h"
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

  pinModeMCP23S17(BUTTON_0_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_1_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_2_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_3_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_4_PIN,             INPUT_PULLUP);
  pinModeMCP23S17(BUTTON_5_PIN,             INPUT_PULLUP);
  pinModeMCP23S17(BUTTON_6_PIN,             INPUT_PULLUP);
  pinModeMCP23S17(BUZZER_PIN,               OUTPUT);
  pinModeMCP23S17(MOSFET_NIGH_LIGHT_PIN,    OUTPUT);
  pinModeMCP23S17(MOSFET_BRAKE_LIGHT_PIN,   OUTPUT);
  pinModeMCP23S17(MOSFET_HORN_PIN,          OUTPUT);
  pinModeMCP23S17(MOSFET_REVERSE_PIN,       OUTPUT);
  pinModeMCP23S17(MOSFET_BRAKE_PIN,         OUTPUT);
  pinModeMCP23S17(SENSOR_WHEEL_SPEED_PIN,   INPUT_PULLDOWN);
  pinModeMCP23S17(SENSOR_5V_EMPTY_PIN,      INPUT_PULLDOWN);
  pinModeMCP23S17(SENSOR_PEDAL_TRIGGER_PIN, INPUT_PULLDOWN);


  digitalWriteMCP23S17(BUZZER_PIN,             LOW);
  digitalWriteMCP23S17(MOSFET_BRAKE_LIGHT_PIN, LOW);
  digitalWriteMCP23S17(MOSFET_HORN_PIN,        LOW);
  digitalWriteMCP23S17(MOSFET_NIGH_LIGHT_PIN,  LOW);
  digitalWriteMCP23S17(MOSFET_BRAKE_PIN,       LOW);
  digitalWriteMCP23S17(MOSFET_REVERSE_PIN,     LOW);

  //readMCP23S17(0x00) always reads 0 see there is no way for us to check if the module is working correctly ;(
  // Check if the module is initialized (simple check), Check IODIR register
  //if (readMCP23S17(0x00) == 0x00) 
  //{  
  //  WiFiPrinter::print("MCP23S17 Initialization Failed!");
  //} 
  //else 
  //{
  unsigned long initialization_time = millis() - module_connection_time_Start;
  WiFiPrinter::print("MCP23S17 initialized in " + String( initialization_time ) + "ms");
  initialized = true;
  //} 

  // Deactivate SPI device
  digitalWrite(CS_PIN, HIGH);
}

// Deinitialize the MCP23S17
void PortExpander::shutdown()
{
  if( !initialized )
  {
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);  // Keep CS pin high initially
  }

  // Set all MCP23S17 GPIOs to INPUT mode
  pinModeMCP23S17(BUTTON_0_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_1_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_2_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_3_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_4_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_5_PIN,             INPUT);
  pinModeMCP23S17(BUTTON_6_PIN,             INPUT);
  pinModeMCP23S17(BUZZER_PIN,               INPUT);
  pinModeMCP23S17(MOSFET_NIGH_LIGHT_PIN,    INPUT);
  pinModeMCP23S17(MOSFET_BRAKE_LIGHT_PIN,   INPUT);
  pinModeMCP23S17(MOSFET_HORN_PIN,          INPUT);
  pinModeMCP23S17(SENSOR_WHEEL_SPEED_PIN,   INPUT);
  pinModeMCP23S17(SENSOR_5V_EMPTY_PIN,      INPUT);
  pinModeMCP23S17(SENSOR_PEDAL_TRIGGER_PIN, INPUT);
  pinModeMCP23S17(MOSFET_REVERSE_PIN,       INPUT);
  pinModeMCP23S17(MOSFET_BRAKE_PIN,         INPUT);

  writeMCP23S17(0x12, 0x00); // Clear GPIOA
  writeMCP23S17(0x13, 0x00); // Clear GPIOB
  

  // Disable SPI communication by setting CS pin HIGH and then INPUT
  digitalWrite(CS_PIN, HIGH); // Set CS HIGH
  pinMode(CS_PIN, INPUT);     // Tri-state CS pin

  // Optional: Set other SPI pins (MOSI, MISO, SCK) to INPUT
  pinMode(MOSI, INPUT);
  pinMode(MISO, INPUT);
  pinMode(SCK, INPUT);

  // Wait for a brief moment to stabilize
  delay(10);

  initialized = false;
}

// Update function (implement as needed)
void PortExpander::update() 
{
}

// Private methods for MCP23S17 operations
void PortExpander::writeMCP23S17(uint8_t registerAddress, uint8_t data) {
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x40); // Write command
    SPI.transfer(registerAddress);
    SPI.transfer(data);
    digitalWrite(CS_PIN, HIGH);
}

uint8_t PortExpander::readMCP23S17(uint8_t registerAddress) 
{
    digitalWrite(CS_PIN, LOW);
    SPI.transfer(0x41); // Read command
    SPI.transfer(registerAddress);
    uint8_t data = SPI.transfer(0x00);
    digitalWrite(CS_PIN, HIGH);
    return data;
}

void PortExpander::pinModeMCP23S17(const PortExpanderPin& pin, uint8_t mode) {
    uint8_t registerAddress = (pin.port == 'A') ? 0x00 : 0x01; // IODIRA or IODIRB
    uint8_t currentIODIR = readMCP23S17(registerAddress);
    if (mode == INPUT) {
        currentIODIR |= (1 << pin.pin);
    } else {
        currentIODIR &= ~(1 << pin.pin);
    }
    writeMCP23S17(registerAddress, currentIODIR);
}

void PortExpander::pullUpMCP23S17(const PortExpanderPin& pin, bool enable) {
    uint8_t registerAddress = (pin.port == 'A') ? 0x0C : 0x0D; // GPPUA or GPPUB
    uint8_t currentGPPU = readMCP23S17(registerAddress);
    if (enable) {
        currentGPPU |= (1 << pin.pin);
    } else {
        currentGPPU &= ~(1 << pin.pin);
    }
    writeMCP23S17(registerAddress, currentGPPU);
}

uint8_t PortExpander::digitalReadMCP23S17(const PortExpanderPin& pin) 
{
    uint8_t registerAddress = (pin.port == 'A') ? 0x12 : 0x13; // GPIOA or GPIOB
    uint8_t currentGPIO = readMCP23S17(registerAddress);
    return (currentGPIO & (1 << pin.pin)) ? HIGH : LOW;
}

void PortExpander::digitalWriteMCP23S17(const PortExpanderPin& pin, uint8_t value) {
    uint8_t registerAddress = (pin.port == 'A') ? 0x12 : 0x13; // GPIOA or GPIOB
    uint8_t currentGPIO = readMCP23S17(registerAddress);
    if (value == HIGH) {
        currentGPIO |= (1 << pin.pin);
    } else {
        currentGPIO &= ~(1 << pin.pin);
    }
    writeMCP23S17(registerAddress, currentGPIO);
}
