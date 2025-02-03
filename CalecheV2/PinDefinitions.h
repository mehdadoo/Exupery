#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// Pin definitions for I2C
#define I2C_SDA 35 
#define I2C_SCL 36

// Pin definitions for existing functionality
#define POWER_SWITCH_PIN  14  // Input for car key switch
#define MOSFET_48V_PIN      34   // Output for 48V MOSFET

#define VOLTAGE_DIRECT_PIN 15

/*
Servo 1: ledc 0, Channel: 0, Timer: 0
Servo 2: ledc 1, Channel: 1, Timer: 0
Servo 3: ledc 2, Channel: 2, Timer: 1
Voltmeter 1: ledc 5, Channel: 5, Timer: 2
Voltmeter 2: ledc 5, Channel: 6, Timer: 2
Voltmeter 3: ledc 5, Channel: 7, Timer: 3
*/

// voltmeter
#define VOLTMETER_SPEED            1       
#define VOLTMETER_CHARGING         5   
#define VOLTMETER_BATTERY          3   
#define VOLTMETER_SPEED_CHANNEL    5
#define VOLTMETER_CHARGING_CHANNEL 7
#define VOLTMETER_BATTERY_CHANNEL  6

#define SERVO_BRAKE_1   7
#define SERVO_BRAKE_2   17
#define SERVO_STEERING  21

#define DigiPot_NC 16
#define DigiPot_UD 18
#define DigiPot_CS1 8
#define DigiPot_CS2 9

// Pin definitions for LCD
#define TFT_CS    2 
#define TFT_DC    4
#define TFT_RST   6
#define TFT_SCK   12  //SPI scl
#define TFT_MOSI  13  //SPI sda


// Pin Definitions for SPI Communication
#define SCK_PIN 12   // Clock
#define MOSI_PIN 13  // Master Out Slave In
#define MISO_PIN 37  // Master In Slave Out


//port expander
#define CS_PIN 38    // Chip Select
#define MCP23S17_INT_PIN 10


struct PortExpanderPin 
{
    uint8_t port;
    uint8_t pin;
};

const PortExpanderPin BUTTON_0_PIN              = {'B', 0};
const PortExpanderPin BUTTON_1_PIN              = {'B', 1};
const PortExpanderPin BUTTON_2_PIN              = {'B', 2};
const PortExpanderPin BUTTON_3_PIN              = {'B', 3};
const PortExpanderPin BUTTON_4_PIN              = {'B', 4};
const PortExpanderPin BUTTON_5_PIN              = {'B', 5};
const PortExpanderPin BUTTON_6_PIN              = {'B', 6};
const PortExpanderPin BUZZER_PIN                = {'B', 7};
const PortExpanderPin MOSFET_NIGH_LIGHT_PIN     = {'A', 0};
const PortExpanderPin MOSFET_BRAKE_LIGHT_PIN    = {'A', 1};
const PortExpanderPin MOSFET_HORN_PIN           = {'A', 2};
const PortExpanderPin SENSOR_WHEEL_SPEED_PIN    = {'A', 3};
const PortExpanderPin SENSOR_5V_EMPTY_PIN       = {'A', 4};
const PortExpanderPin SENSOR_PEDAL_TRIGGER_PIN  = {'A', 5};
const PortExpanderPin MOSFET_REVERSE_PIN        = {'A', 6};
const PortExpanderPin MOSFET_BRAKE_PIN          = {'A', 7};

#endif