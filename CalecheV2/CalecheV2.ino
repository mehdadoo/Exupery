#include <Arduino.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <DigiPotX9Cxxx.h>

#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "WiFiPrinter.h"
#include "PortExpander.h"
#include "IgnitionSwitch.h"
#include "VoltageSensor.h"
#include "InclinationSensor.h"
#include "SpeedSensor.h"
#include "PedalSensor.h"
#include "BrakeSystem.h"
#include "ThrottleSystem.h"
#include "Dashboard.h"



PortExpander& portExpander = PortExpander::getInstance();//MPC23S17 port expander
VoltageSensor voltageSensor;//ADS1115 current and voltage reader
IgnitionSwitch ignitionSwitch;
Dashboard dashboard;
InclinationSensor inclinationSensor;
SpeedSensor speedSensor;
PedalSensor pedalSensor;
BrakeSystem brakeSystem(speedSensor); // Pass speedSensor to the constructor
ThrottleSystem throttleSystem(dashboard, pedalSensor);

//servos
Servo servoSteering; 


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
  pedalSensor.update();
  voltageSensor.update();
  
  if( dashboard.initialized )
  {
    dashboard.batteryPercentage = voltageSensor.batteryPercentage;
    dashboard.update();
    updateServos();
    brakeSystem.update();
    throttleSystem.update();
  }
  
  updateOverHTTP();
}

void start()
{
  SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN);
  portExpander.start();
  voltageSensor.start();
  dashboard.start();
  brakeSystem.start();
  throttleSystem.start();
}

void shutdown()
{
  throttleSystem.shutdown();
  brakeSystem.shutdown();
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
                          speedSensor.getSpeed(), speedSensor.getRPM(),
                          dashboard.joystick_throttle, dashboard.joystick_knob,  dashboard.joystick_steering,
                          voltageSensor.voltage,
                          voltageSensor.current,
                          inclinationSensor.getInclinationAngle() );
  }

  WiFiPrinter::update();
}


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