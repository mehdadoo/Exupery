#include "Gearbox.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

Gearbox::Gearbox(SpeedSensor& sensorInstance)
    : speedSensor(sensorInstance) // Initialize the speedSensor member
{
}


// Update the gearbox
void Gearbox::update() 
{
    static unsigned long lastUpdateTime = 0; // Tracks the last time the method was called
    unsigned long currentTime = millis();

    // Check if 230 milliseconds have passed
    if (currentTime - lastUpdateTime >= SPEED_UPDATE_FREQUENCY) 
    {
        lastUpdateTime = currentTime; // Update the last update time

        // Send RPM and speed to WiFiPrinter
        WiFiPrinter::print(RPM_CONSTANT, speedSensor.getRPM());
        WiFiPrinter::print(SPEED_CONSTANT, speedSensor.getSpeed());
    }
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
  WiFiPrinter::print(CUSTOM_MESSAGE, "Gearbox setup complete!"); 
}
