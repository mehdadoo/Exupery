#include "BrakeSystem.h"
#include "ConstantDefinitions.h"
#include "WiFiPrinter.h"
#include <algorithm> // For std::clamp

// Global instance for rotary callback
static BrakeSystem* instance = nullptr;

// Constructor
BrakeSystem::BrakeSystem(SpeedSensor& sensorInstance)
    : speedSensor(sensorInstance), // Assign passed instance to private property
      rotary(onBrakeLeverPositionChange, RotaryEncoder_DT, RotaryEncoder_CLK, RotaryEncoder_SW),
      brakeLeverPosition(0) 
{
    // Set the global instance to this object
    instance = this;

    // Initialize brake lever position
    brakeLeverPosition = 0;

    // Calculate the threshold range
    lowerLeverThreshold = 0 + MIN_BRAKE_LEVER_THRESHOLD;
    upperLeverThreshold = MAX_BRAKE_LEVER_STEPS + MIN_BRAKE_LEVER_THRESHOLD;
}



void BrakeSystem::update() 
{
  if( brakeSystemUpToDate )
    return;

  updateBrakeLights();
  updateServo();

  // Check if the brakeLeverPosition is below the threshold to stop updating the servo and lights
  if (brakeLeverPosition <= lowerLeverThreshold) 
    brakeSystemUpToDate = true;
}

//Update using a ease method, so that at lower positions of the lever, the servo reacts 3 times more compared to the highest lever position
void BrakeSystem::updateServo()
{
  if (brakeLeverPosition >= lowerLeverThreshold && brakeLeverPosition <= upperLeverThreshold) 
  {
      // Normalize brakeLeverPosition to a 0-1 range
      float normalizedPosition = 
          (float)(brakeLeverPosition - lowerLeverThreshold) / (upperLeverThreshold - lowerLeverThreshold);

      // Apply 3x ease-out (cubic root)
      float easedPosition = pow(normalizedPosition, 1.0 / 3.0);

      // Map easedPosition to servo angle range
      int servoPosition = MIN_BRAKE_ANGLE + 
          (int)(easedPosition * (MAX_BRAKE_ANGLE - MIN_BRAKE_ANGLE));

      // Move servo
      servo.write(servoPosition);

      WiFiPrinter::print(BRAKE_SERVO_CONSTANT, servoPosition); 
  }
}

void BrakeSystem::updateBrakeLights()
{
    static unsigned long positionInRangeStartTime = 0; // Time when lever entered the range
    static bool lightState = false; // State of the light
    static unsigned long lastBlinkTime = 0; // Last time the light blinked

    unsigned long currentTime = millis();

    // Check if the brake lever is below the lower threshold to turn off the light
    if (brakeLeverPosition <= lowerLeverThreshold) 
    {
        digitalWrite(LIGHT_BRAKE_PIN, LOW); // Brake light off
        positionInRangeStartTime = 0; // Reset the timer since it's out of range
    } 
    else if (brakeLeverPosition <= upperLeverThreshold) 
    {
        // Check if the lever has been in the range for DELAY_BEFORE_FULL_BRAKE_BRIGHTNESS
        if (positionInRangeStartTime == 0) 
        {
            positionInRangeStartTime = currentTime; // Start timing
        }

        if (currentTime - positionInRangeStartTime > DELAY_BEFORE_FULL_BRAKE_BRIGHTNESS) 
        {
            // Full brightness: Keep the light on
            digitalWrite(LIGHT_BRAKE_PIN, HIGH);
            lightState = true; // Ensure state is consistent
        } 
        else 
        {
            // Calculate the blink interval based on the lever position
            int range = upperLeverThreshold - lowerLeverThreshold;
            int positionInRange = brakeLeverPosition - lowerLeverThreshold;
            int multiplier = range == 0 ? 1 : map(positionInRange, 0, range, BRAKE_BLINK_MULTIPLIER, 1); // Inverse multiplier from 10 to 1
            int blinkInterval = multiplier * BRAKE_BLINK_RATE;

            // Blink the brake light with the calculated interval
            if (currentTime - lastBlinkTime >= blinkInterval) 
            {
                lightState = !lightState; // Toggle the light state
                digitalWrite(LIGHT_BRAKE_PIN, lightState ? HIGH : LOW);
                lastBlinkTime = currentTime; // Reset the timer
            }
        }
    } 
    else 
    {
        // Reset the timer if the lever position leaves the range
        positionInRangeStartTime = 0;
    }
}







// Static callback function for rotary encoder
void BrakeSystem::onBrakeLeverPositionChange() 
{
    // Ensure we have an instance to work with
    if (instance) 
    {
        const unsigned int state = instance->rotary.GetState();

        if (state & DIR_CW)
            instance->brakeLeverPosition ++;

        if (state & DIR_CCW)
            instance->brakeLeverPosition --;

        // Clamp brake lever position
        instance->brakeLeverPosition = std::clamp(instance->brakeLeverPosition, 0, MAX_BRAKE_LEVER_STEPS);

        instance->brakeSystemUpToDate = false;

        WiFiPrinter::print(BRAKE_LEVER_CONSTANT, instance->brakeLeverPosition); 
    }
}


/////////////////////////////////////////////////////////////////////
////////////////////          Setup         /////////////////////////
/////////////////////////////////////////////////////////////////////

void BrakeSystem::setup() 
{
  pinMode(LIGHT_BRAKE_PIN, OUTPUT);
  rotary.setup();
  setupServo();
  
  Serial.println("BrakeSystem setup complete!");
  WiFiPrinter::print(CUSTOM_MESSAGE, "BrakeSystem setup complete!");
}

void BrakeSystem::setupServo() 
{
  servo.attach( SERVO_BRAKE_PIN );
  delay(300);

  update();
}
