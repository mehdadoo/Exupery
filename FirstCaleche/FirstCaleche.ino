#include "PinDefinitions.h"
#include "Gearbox.h"
#include "BrakeSystem.h"
#include "NightLights.h"
#include "WiFiPrinter.h"
#include <ESP32Servo.h>

//#define DEBUG_MODE


SpeedSensor speedSensor;              // Create the SpeedSensor instance
BrakeSystem brakeSystem(speedSensor); // Pass speedSensor to the constructor
Gearbox     gearbox(speedSensor); //Gearbox 
NightLights nightLights; // Pass speedSensor to the  onstructor


void setup()
{
  setupSerial();
  speedSensor.setup();
  gearbox.setup();
  brakeSystem.setup();
  nightLights.setup();

  WiFiPrinter::print("Setup Complete!");
}

void setupSerial()
{
  #ifndef DEBUG_MODE
    return;
  #endif


  Serial.begin(9600);

  // Check if the serial port is available
  unsigned long startMillis = millis();
  while (!Serial && millis() - startMillis < 2000) 
  {
    // Wait up to 5 seconds for the serial connection
    delay(10);
  }
  Serial.println( "Serial startup: " + String ( millis() - startMillis ) );
  

  WiFiPrinter::setup();
  WiFiPrinter::print("First Calèche, Bonjour!");
}


void updateOverHTTP()
{
  static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
  unsigned long currentTime = millis();

  // Check if enough time has been passed since last print call
  if (currentTime - lastUpdateTime >= UPDATE_OVER_HTTP_FREQUENCY) 
  {
      lastUpdateTime = currentTime; // Update the last update time
      WiFiPrinter::printAll( speedSensor.getRPM(), speedSensor.getSpeed(), brakeSystem.brakeLeverPosition, brakeSystem.servoPosition, gearbox.gear, gearbox.servoPosition );
  }

  WiFiPrinter::update();
}


void loop()
{
  speedSensor.update();
  gearbox.update();
  brakeSystem.update();
  nightLights.update();

  updateOverHTTP();
}