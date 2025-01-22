#include "Gearbox.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"


Gearbox::Gearbox(SpeedSensor& sensorInstance)
    : speedSensor(sensorInstance), 
      gear(1), 
      servoPosition(MIN_GEARBOX_ANGLE), 
      lastGearChangeDirection(UPSHIFT), // Initialize lastDirection
      oneGearPortion((LAST_GEAR_SPEED - FIRST_GEAR_SPEED) / (TOTAL_GEARS - 2)) // Compute portion directly
{}



// Update the gearbox
void Gearbox::update() 
{
    if (updateGear()) 
    {
        updateServo(); // Update the servo position only if the gear has changed
    }
}


int Gearbox::calculateGearRatio()
{
    float speed = speedSensor.getSpeed(); // Get the current speed

    // Determine the gear based on speed
    if (speed < FIRST_GEAR_SPEED) 
    {
        return 1;
    } 
    else if (speed > LAST_GEAR_SPEED) 
    {
        return TOTAL_GEARS;
    } 
    else 
    {
        // Calculate the gear for speeds in between
        return 2 + (int)((speed - FIRST_GEAR_SPEED) / oneGearPortion);
    }
}

// Check if enough time has passed since the last gear shift
bool Gearbox::isShiftCooldownComplete() 
{
  if (millis() - lastShiftTime < SHIFT_COOLDOWN_TIME) 
    return false;
  else
    return true;
}

// Update the gear based on speed
bool Gearbox::updateGear() 
{
    if ( !isShiftCooldownComplete() ) 
        return false; // Not enough time has passed, no update

    int newGear = calculateGearRatio();

    // Check if the gear has changed
    if (newGear == gear) 
    {
        return false; // Gear remains the same
    }
    else
    {
      // Determine shift direction
      int currentGearChangeDirection = (newGear > gear) ? UPSHIFT : DOWNSHIFT;

      // this mechanism makes sure if the gear direction changes, we add one gear gap in the direction of the new gear change direction so that the gear won't change frequently
      if( currentGearChangeDirection != lastGearChangeDirection)
      {
        if( currentGearChangeDirection == UPSHIFT && abs(newGear - gear) == 1)
          newGear -=1;
        else if( currentGearChangeDirection == DOWNSHIFT && abs(newGear - gear) == 1)
          newGear +=1;
      }

      // we are really changing the gear here finally
      if( newGear != gear )
      {
        gear = newGear;       // Update to the new gear
        lastShiftTime = millis(); // Update the last shift time
        lastGearChangeDirection = currentGearChangeDirection;

        return true;
      }
      else
      {
        return false;
      }
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
