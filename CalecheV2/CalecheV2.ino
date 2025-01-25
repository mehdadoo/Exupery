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
#include "BrakeSystem.h"
#include "PedalSensor.h"



PortExpander& portExpander = PortExpander::getInstance();//MPC23S17 port expander
Dashboard dashboard;
VoltageSensor voltageSensor;//ADS1115 current and voltage reader
IgnitionSwitch ignitionSwitch;
InclinationSensor inclinationSensor;
SpeedSensor speedSensor;
PedalSensor pedalSensor;
BrakeSystem brakeSystem(speedSensor); // Pass speedSensor to the constructor

//servos
Servo servoSteering; 

// Define the digital potentiometer with specified multiplexer channels
DigiPot potentiometer1(DigiPot_NC, DigiPot_UD , DigiPot_CS1);  
DigiPot potentiometer2(DigiPot_NC, DigiPot_UD , DigiPot_CS2);


void setup()
{
  initializeSerial();

  inclinationSensor.start();
  initializeServos();
  brakeSystem.start();

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
  pedalSensor.update();
  voltageSensor.update();
  
  if( dashboard.initialized )
  {
    dashboard.batteryPercentage = voltageSensor.batteryPercentage;
    dashboard.update();
    updateServos();
    brakeSystem.isStopped = speedSensor.isStopped();
    brakeSystem.brakeLeverPosition = dashboard.joystick_throttle;
    brakeSystem.update();
    updatePotentiometers();
  }
  
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
  int servoChannel = servoSteering.attach(SERVO_STEERING);  // Reattach the servo if it was detached
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

void updateOverHTTP()
{
  static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
  unsigned long currentTime = millis();

  // Check if enough time has been passed since last print call
  if (currentTime - lastUpdateTime >= UPDATE_OVER_HTTP_FREQUENCY) 
  {
      lastUpdateTime = currentTime; // Update the last update time
      WiFiPrinter::printAll( ignitionSwitch.isKeyOn,
                          dashboard.toggleState[0], pedalSensor.isStopped(), dashboard.buttonState[2], dashboard.buttonState[3], 
                          speedSensor.getRPM(), speedSensor.getSpeed(),
                          dashboard.joystick_throttle, dashboard.joystick_knob,  dashboard.joystick_steering,
                          voltageSensor.voltage,
                          voltageSensor.current,
                          inclinationSensor.getInclinationAngle() );
  }

  WiFiPrinter::update();
}

/*
void updateServos()
{
  servoSteeringValue = map(dashboard.joystick_steering, JOYSTICK_STEERING_MIN_VALUE, JOYSTICK_STEERING_MAX_VALUE, STERING_SERVO_MIN_VALUE, STERING_SERVO_MAX_VALUE);
  servoSteeringValue = constrain(servoSteeringValue, STERING_SERVO_MIN_VALUE, STERING_SERVO_MAX_VALUE); // Ensure it's within 0-180
  servoSteering.write(servoSteeringValue);

  
}*/

void updateServos()
{
  int joystickMidpoint = (JOYSTICK_STEERING_MIN_VALUE + JOYSTICK_STEERING_MAX_VALUE) / 2; // Middle point of the joystick
  int servoMidpoint = (STERING_SERVO_MIN_VALUE + STERING_SERVO_MAX_VALUE) / 2;           // Middle point of the servo range
  
  if (abs(dashboard.joystick_steering - joystickMidpoint) <= JOYSTICK_STEERING_REST_GAP) 
  {
    // Within the rest gap: set servo to the midpoint
    servoSteeringValue = servoMidpoint;
  } 
  else if (dashboard.joystick_steering < joystickMidpoint - JOYSTICK_STEERING_REST_GAP) 
  {
    // Left portion of the joystick: map to the left servo range
    servoSteeringValue = map(dashboard.joystick_steering, 
                             JOYSTICK_STEERING_MIN_VALUE, 
                             joystickMidpoint - JOYSTICK_STEERING_REST_GAP, 
                             STERING_SERVO_MIN_VALUE, 
                             servoMidpoint);
  } 
  else 
  {
    // Right portion of the joystick: map to the right servo range
    servoSteeringValue = map(dashboard.joystick_steering, 
                             joystickMidpoint + JOYSTICK_STEERING_REST_GAP, 
                             JOYSTICK_STEERING_MAX_VALUE, 
                             servoMidpoint, 
                             STERING_SERVO_MAX_VALUE);
  }

  // Ensure the calculated servo value is within the allowed range
  servoSteeringValue = constrain(servoSteeringValue, STERING_SERVO_MIN_VALUE, STERING_SERVO_MAX_VALUE);

  // Write the value to the servo
  servoSteering.write(servoSteeringValue);
}



void updatePotentiometers()
{
  if( !dashboard.initialized )
    return;

  int joystick_knob = dashboard.joystick_knob;
  int joystick_throttle = dashboard.joystick_throttle;


  potValue1 = map(joystick_knob, 0, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);
  potValue2 = 0; // Initialize potValue2

  if (joystick_throttle < JOYSTICK_THROTTLE_REST_MAX) 
      potValue2 = 0; // Set potValue2 to 0 if below 100
  else
      // Map joystick_throttle from 100 to 200 to potValue2 from 0 to 75
      potValue2 = map(joystick_throttle, JOYSTICK_THROTTLE_REST_MAX, JOYSTICK_THROTTLE_MAX_VALUE, POTENTIOMETER_MIN_VALUE, POTENTIOMETER_MAX_VALUE);


  // Ensure potValue2 never exceeds 75
  potValue1 = min(potValue1, POTENTIOMETER_MAX_VALUE);
  potValue2 = min(potValue2, POTENTIOMETER_MAX_VALUE);

 // Apply the value to the potentiometers
  potentiometer1.set(potValue2);
  potentiometer2.set(potValue1);
}