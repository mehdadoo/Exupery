#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// Pin definitions for I2C
#define I2C_SDA 35 
#define I2C_SCL 36

// Pin definitions for existing functionality
#define CAR_KEY_SWITCH 15   // Input for car key switch
#define MOSFET_48V 34       // Output for 48V MOSFET

#define VOLTMETER_SPEED 1       
#define VOLTMETER_CHARGING 3   
#define VOLTMETER_BATTERY 5   


#define SERVO_BRAKE_1 7
#define SERVO_BRAKE_2 17
#define SERVO_STEERING 21

#define DistanceSensor_trigPin 37
#define DistanceSensor_echoPin 38

#define DigiPot_NC 16
#define DigiPot_UD 18
#define DigiPot_CS1 8
#define DigiPot_CS2 9

// Pin definitions for LCD
#define TFT_CS 2 
#define TFT_DC 4
#define TFT_RST 6
#define TFT_SCK 12
#define TFT_MOSI 13

#endif