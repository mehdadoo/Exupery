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

//port expander cs pin
#define CS_PIN 38    // Chip Select

// GPIO Pin Definitions for Buttons (GPB0 to GPB3)
#define BUTTON_0_PIN  0   // GPB0 (Pin 0 on Bank B)
#define BUTTON_1_PIN  1   // GPB1 (Pin 1 on Bank B)
#define BUTTON_2_PIN  2   // GPB2 (Pin 2 on Bank B)
#define BUTTON_3_PIN  3   // GPB3 (Pin 3 on Bank B)
#define BUTTON_4_PIN  4   // GPB4 (Pin 3 on Bank B)
#define BUTTON_5_PIN  5   // GPB5 (Pin 3 on Bank B)
#define BUTTON_6_PIN  6   // GPB6 (Pin 3 on Bank B)
#define BUZZER_PIN    7   // GPB7 (Pin 7 on Bank B)

// GPIO Pin Definitions for MOSFET Control
#define MOSFET_NIGH_LIGHT_PIN     0  // GPIO A0 (Pin 0 on Bank A)
#define MOSFET_BRAKE_LIGHT_PIN    1  // GPIO A1 (Pin 1 on Bank A)
#define MOSFET_HORN_PIN           2  // GPIO A2 (Pin 2 on Bank A)
#define SENSOR_WHEEL_SPEED_PIN    3  //GPIO A3 (Pin 3 on Bank A)
#define SENSOR_5V_EMPTY_PIN       4  //GPIO A4 (Pin 4 on Bank A)
#define SENSOR_PEDAL_TRIGGER_PIN  5  //GPIO A5 (Pin 5 on Bank A)
#define MOSFET_REVERSE_PIN        6  // GPIO A6 (Pin 6 on Bank A)
#define MOSFET_BRAKE_PIN          7  // GPIO A7 (Pin 7 on Bank A)

// GPIO Pin Definitions for Sensors







#endif