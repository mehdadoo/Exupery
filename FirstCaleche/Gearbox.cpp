#include "Gearbox.h"
#include "WiFiPrinter.h"

Gearbox::Gearbox(SpeedSensor& sensorInstance)
    : speedSensor(sensorInstance) // Initialize the speedSensor member
{
}


// Update the gearbox
void Gearbox::update() 
{
  
}

void Gearbox::updateServo()
{
  
}



/////////////////////////////////////////////////////////////////////
////////////////////          Setup         /////////////////////////
/////////////////////////////////////////////////////////////////////

void Gearbox::setup() 
{
  servoGearbox.attach( SERVO_GEARBOX_PIN );
  delay(300);
  servoGearbox.write( MIN_GEARBOX_ANGLE );

  Serial.println("Gearbox setup complete!");
  WiFiPrinter::print(CUSTOM_MESSAGE, "Gearbox setup complete!"); 
}
