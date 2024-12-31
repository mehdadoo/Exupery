#include "Gearbox.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

Gearbox::Gearbox(SpeedSensor& sensorInstance)
    : speedSensor(sensorInstance), gear(1), servoPosition(MIN_GEARBOX_ANGLE) // Initialize gear and servoPosition
{}


// Update the gearbox
void Gearbox::update() 
{
    static unsigned long lastShiftTime = 0; // To track the last gear shift time
    unsigned long currentTime = millis();

    // Check if enough time has passed since the last gear shift
    if (currentTime - lastShiftTime >= MIN_TIME_BETWEEN_SHIFTS) 
    {
        float speed = speedSensor.getSpeed(); // Get the current speed

        // Determine the gear based on speed
        if (speed < FIRST_GEAR_SPEED) 
        {
            gear = 1;
        } 
        else if (speed > LAST_GEAR_SPEED) 
        {
            gear = TOTAL_GEARS;
        } 
        else 
        {
            // Calculate the gear for speeds in between
            float speedRange = LAST_GEAR_SPEED - FIRST_GEAR_SPEED;
            float portion = speedRange / (TOTAL_GEARS - 2); // Divide into 6 portions
            gear = 2 + (int)((speed - FIRST_GEAR_SPEED) / portion);
        }

        // Update servo position
        updateServo();

        // Update the last shift time
        lastShiftTime = currentTime;
    }
}

// Update the servo position based on the current gear
void Gearbox::updateServo()
{
    // Map the gear to the servo position
    int gearRange = TOTAL_GEARS - 1; // Total gear transitions
    int angleRange = MAX_GEARBOX_ANGLE - MIN_GEARBOX_ANGLE; // Servo angle range

    servoPosition = MIN_GEARBOX_ANGLE + (gear - 1) * (angleRange / gearRange);

    // Ensure the servo position stays within bounds
    servoPosition = constrain(servoPosition, MAX_GEARBOX_ANGLE, MIN_GEARBOX_ANGLE);

    // Move the servo
    servoGearbox.write(servoPosition);
}



/////////////////////////////////////////////////////////////////////
////////////////////          Setup         /////////////////////////
/////////////////////////////////////////////////////////////////////

void Gearbox::setup() 
{
  servoGearbox.attach( SERVO_GEARBOX_PIN );
  delay(300);

  servoGearbox.write( MIN_GEARBOX_ANGLE );
  WiFiPrinter::print("Gearbox setup complete!"); 
}
