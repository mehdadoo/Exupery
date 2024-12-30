#include "PinDefinitions.h"
#include "Gearbox.h"
#include "BrakeSystem.h"
#include "NightLights.h"
#include "WiFiPrinter.h"
#include <ESP32Servo.h>


SpeedSensor speedSensor;              // Create the SpeedSensor instance
BrakeSystem brakeSystem(speedSensor); // Pass speedSensor to the constructor
Gearbox     gearbox(speedSensor); //Gearbox 
NightLights nightLights; // Pass speedSensor to the  onstructor


void setup()
{
  WiFiPrinter::setup();

  WiFiPrinter::print(CUSTOM_MESSAGE, "First Caleche, Bonjour!");

  speedSensor.setup();
  gearbox.setup();
  brakeSystem.setup();
  nightLights.setup();

  WiFiPrinter::print(CUSTOM_MESSAGE, "Setup Complete!");
}


void loop()
{
  WiFiPrinter::update();
  speedSensor.update();
  gearbox.update();
  brakeSystem.update();
  nightLights.update();
}