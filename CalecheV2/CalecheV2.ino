#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <DigiPotX9Cxxx.h>
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "Dashboard.h"
#include "VoltageSensor.h"
#include "IgnitionSwitch.h"
#include "InclinationSensor.h"
#include "SpeedSensor.h"


PortExpander& portExpander = PortExpander::getInstance();//MPC23S17 port expander
Dashboard dashboard;
VoltageSensor voltageSensor;//ADS1115 current and voltage reader
IgnitionSwitch ignitionSwitch;
InclinationSensor inclinationSensor;
SpeedSensor speedSensor;

//servos
Servo servoBrake1; 
Servo servoBrake2; 
Servo servoSteering; 

// Define the digital potentiometer with specified multiplexer channels
DigiPot potentiometer1(DigiPot_NC, DigiPot_UD , DigiPot_CS1);  
DigiPot potentiometer2(DigiPot_NC, DigiPot_UD , DigiPot_CS2);


void setup()
{
  initializeSerial();

  inclinationSensor.start();
  initializeServos();

  // Set event listeners
  ignitionSwitch.setOnTurnedOnListener([]() {
    start();
  });

  ignitionSwitch.setOnTurnedOffListener([]() {
      shutdown();
  });

  ignitionSwitch.setup();
}

void loop()
{
  ignitionSwitch.update();
  inclinationSensor.update();
  speedSensor.update();
  voltageSensor.update();
  dashboard.batteryPercentage = voltageSensor.batteryPercentage;
  dashboard.update();
  
  updateServos();
  updatePotentiometers();
  updateOverHTTP();
}

void start()
{
  // Initialize SPI Communication for display and port expander
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  portExpander.start();
  voltageSensor.start();
  dashboard.start();
}


void shutdown()
{
  potentiometer1.set(0);
  potentiometer2.set(0);

  dashboard.shutdown();
  voltageSensor.shutdown();
  portExpander.shutdown();

  SPI.end();
}

void initializeSerial()
{
  Serial.begin(9600);
  
  WiFiPrinter::setup();
  WiFiPrinter::print("Blue Calèche, Bonjour!");
}


void initializeServos() 
{
  int servoChannel = servoBrake1.attach(SERVO_BRAKE_1);  // Reattach the servo if it was detached
  Serial.println("servoBrake1 Channel:" + String(servoChannel));

  servoChannel = servoBrake2.attach(SERVO_BRAKE_2);  // Reattach the servo if it was detached
  Serial.println("servoBrake2 Channel:" + String(servoChannel));

  servoChannel = servoSteering.attach(SERVO_STEERING);  // Reattach the servo if it was detached
  Serial.println("servoSteering Channel:" + String(servoChannel));
  WiFiPrinter::print("servoSteering Channel:" + String(servoChannel));

  delay(50);
}

void initializePotentiometers() 
{
  potentiometer1.set(0);
  potentiometer2.set(0);
}


int potValue1 = 0;
int potValue2 = 0;

int servoSteeringValue = 0;

int brake1Value = 0;
int brake2Value = 0;

void updateOverHTTP()
{
  static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
  unsigned long currentTime = millis();

  // Check if enough time has been passed since last print call
  if (currentTime - lastUpdateTime >= UPDATE_OVER_HTTP_FREQUENCY) 
  {
      lastUpdateTime = currentTime; // Update the last update time
      WiFiPrinter::printAll( ignitionSwitch.isKeyOn,
                          dashboard.buttonState[0], dashboard.buttonState[1], dashboard.buttonState[2], dashboard.buttonState[3], 
                          speedSensor.getRPM(), speedSensor.getSpeed(),
                          dashboard.joystick_throttle, brake1Value /*dashboard.joystick_knob*/, servoSteeringValue /*dashboard.joystick_steering*/,
                          voltageSensor.voltage,
                          voltageSensor.current,
                          inclinationSensor.getInclinationAngle() );
  }

  WiFiPrinter::update();
}


void updateServos()
{
  if( dashboard.initialized )
  {
    int joystick_throttle = dashboard.joystick_throttle;

    if (joystick_throttle >= JOYSTICK_THROTTLE_MIDDLE_VALUE)
    {
        brake1Value = BRAKE_SERVO_MIN_VALUE; // Set brake1Value to max if above 100
        brake2Value = BRAKE_SERVO_MAX_VALUE; // Set brake1Value to 0 if above 100, this servo is in reverse!
    }
    else
    {
        // Map joystick_throttle from 100 to 200 to brake from 0 to 270
        brake1Value = map(joystick_throttle, JOYSTICK_THROTTLE_MIN_VALUE, JOYSTICK_THROTTLE_MIDDLE_VALUE, BRAKE_SERVO_MAX_VALUE, BRAKE_SERVO_MIN_VALUE);
        brake2Value = map(joystick_throttle, JOYSTICK_THROTTLE_MIN_VALUE, JOYSTICK_THROTTLE_MIDDLE_VALUE, BRAKE_SERVO_MIN_VALUE, BRAKE_SERVO_MAX_VALUE);
    }

    servoBrake1.write( brake1Value );
    servoBrake2.write( brake2Value );

    servoSteeringValue = map(dashboard.joystick_steering, 22, 164, 0, 136);
    servoSteeringValue = constrain(servoSteeringValue, 0, 136); // Ensure it's within 0-180
    servoSteering.write(servoSteeringValue);
  }
}


void updatePotentiometers()
{
  if( !dashboard.initialized )
    return;

  int joystick_knob = dashboard.joystick_knob;
  int joystick_throttle = dashboard.joystick_throttle;


  potValue1 = map(joystick_knob, 0, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);
  potValue2 = 0; // Initialize potValue2

  if (joystick_throttle < JOYSTICK_THROTTLE_MIDDLE_VALUE) 
      potValue2 = 0; // Set potValue2 to 0 if below 100
  else
      // Map joystick_throttle from 100 to 200 to potValue2 from 0 to 75
      potValue2 = map(joystick_throttle, JOYSTICK_THROTTLE_MIDDLE_VALUE, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);


  // Ensure potValue2 never exceeds 75
  potValue1 = min(potValue1, POTENTIOMETER_MAX_VALUE);
  potValue2 = min(potValue2, POTENTIOMETER_MAX_VALUE);

 // Apply the value to the potentiometers
  potentiometer1.set(potValue2);
  potentiometer2.set(potValue1);
}


void registerError(const char* errorMessage)
{
    WiFiPrinter::print(errorMessage);
}