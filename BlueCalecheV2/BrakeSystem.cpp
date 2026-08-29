#include "BrakeSystem.h"
#include "ConstantDefinitions.h"
#include "WiFiPrinter.h"

// Constructor
BrakeSystem::BrakeSystem(SpeedSensor& sensorInstance)
    : brakeLeverPosition(0),
      brakeLeverRawValue(0),
      frontServoPosition(BRAKE_SERVO_1_MIN_VALUE),
      backServoPosition(BRAKE_SERVO_2_MIN_VALUE),
      handBrakeEnabled(false),
      simulatedBrakeLeverEnabled(false),
      lowerLeverThreshold(MIN_BRAKE_LEVER_THRESHOLD),
      upperLeverThreshold(BRAKE_LEVER_RESOLUTION),
      filteredPotReading(0),
      lastPotSampleTime(0),
      potFilterInitialized(false),
      lastHandBrakeButtonReading(HIGH),
      stableHandBrakeButtonState(HIGH),
      handBrakeDebounceStartTime(0),
      brakeSystemUpToDate(false),
      speedSensor(sensorInstance)
{
}

void BrakeSystem::update() 
{
  updateHandBrakeButton();

  int newBrakeLeverPosition = simulatedBrakeLeverEnabled
      ? brakeLeverPosition
      : readBrakeLeverPosition();
  if (newBrakeLeverPosition != brakeLeverPosition)
  {
    brakeLeverPosition = newBrakeLeverPosition;
    brakeSystemUpToDate = false;
  }

  if( brakeSystemUpToDate )
    return;

  updateBrakeLights();
  updateServo();

  // Check if the brakeLeverPosition is below the threshold to stop updating the servo and lights
  if (brakeLeverPosition <= lowerLeverThreshold) 
    brakeSystemUpToDate = true;
}

void BrakeSystem::updateHandBrakeButton()
{
  if (!ENABLE_HAND_BRAKE_BUTTON)
    return;

  bool buttonReading = digitalRead(HAND_BRAKE_BUTTON_PIN);
  unsigned long currentTime = millis();

  if (buttonReading != lastHandBrakeButtonReading)
  {
    lastHandBrakeButtonReading = buttonReading;
    handBrakeDebounceStartTime = currentTime;
  }

  if (currentTime - handBrakeDebounceStartTime >= HAND_BRAKE_DEBOUNCE_MS &&
      buttonReading != stableHandBrakeButtonState)
  {
    stableHandBrakeButtonState = buttonReading;

    if (stableHandBrakeButtonState == LOW)
      toggleHandBrake();
  }
}

void BrakeSystem::toggleHandBrake()
{
  handBrakeEnabled = !handBrakeEnabled;
  digitalWrite(HAND_BRAKE_LED_PIN, handBrakeEnabled ? HIGH : LOW);
  brakeSystemUpToDate = false;
  WiFiPrinter::print(handBrakeEnabled ? "Handbrake ON" : "Handbrake OFF");
}

void BrakeSystem::setSimulatedBrakeLeverPosition(int position)
{
  simulatedBrakeLeverEnabled = true;
  brakeLeverPosition = constrain(position, 0, BRAKE_LEVER_RESOLUTION);
  brakeSystemUpToDate = false;
}

void BrakeSystem::usePhysicalBrakeLever()
{
  simulatedBrakeLeverEnabled = false;
  potFilterInitialized = false;
  brakeSystemUpToDate = false;
}

int BrakeSystem::readBrakeLeverPosition()
{
  unsigned long currentTime = millis();
  if (potFilterInitialized &&
      currentTime - lastPotSampleTime < BRAKE_POT_SAMPLE_INTERVAL)
  {
    return brakeLeverPosition;
  }
  lastPotSampleTime = currentTime;

  brakeLeverRawValue = analogRead(BRAKE_LEVER_POT_PIN);
  int potentiometerReading = constrain(
      brakeLeverRawValue, BRAKE_POT_MIN_READING, BRAKE_POT_MAX_READING);

  if (potentiometerReading <= BRAKE_POT_MIN_READING + BRAKE_POT_DEAD_ZONE)
    potentiometerReading = BRAKE_POT_MIN_READING;
  else if (potentiometerReading >= BRAKE_POT_MAX_READING - BRAKE_POT_DEAD_ZONE)
    potentiometerReading = BRAKE_POT_MAX_READING;

  if (!potFilterInitialized)
  {
    filteredPotReading = potentiometerReading;
    potFilterInitialized = true;
  }
  else
  {
    filteredPotReading =
        (filteredPotReading * (BRAKE_POT_FILTER_SAMPLES - 1) + potentiometerReading)
        / BRAKE_POT_FILTER_SAMPLES;
  }

  potentiometerReading = filteredPotReading;

  if (potentiometerReading <= BRAKE_POT_MIN_READING + BRAKE_POT_DEAD_ZONE)
    potentiometerReading = BRAKE_POT_MIN_READING;
  else if (potentiometerReading >= BRAKE_POT_MAX_READING - BRAKE_POT_DEAD_ZONE)
    potentiometerReading = BRAKE_POT_MAX_READING;

  if (BRAKE_POT_REVERSED)
  {
    potentiometerReading =
        BRAKE_POT_MAX_READING - (potentiometerReading - BRAKE_POT_MIN_READING);
  }

  return map(
      potentiometerReading,
      BRAKE_POT_MIN_READING,
      BRAKE_POT_MAX_READING,
      0,
      BRAKE_LEVER_RESOLUTION);
}

//Update using a ease method, so that at lower positions of the lever, the servo reacts 3 times more compared to the highest lever position
void BrakeSystem::updateServo()
{
  if (handBrakeEnabled)
  {
    frontServoPosition = BRAKE_SERVO_1_HAND_BRAKE_VALUE;
    backServoPosition = BRAKE_SERVO_2_HAND_BRAKE_VALUE;
  }
  else
  {
    float normalizedPosition = 0.0;
    if (brakeLeverPosition > lowerLeverThreshold)
    {
      normalizedPosition =
          (float)(brakeLeverPosition - lowerLeverThreshold) /
          (upperLeverThreshold - lowerLeverThreshold);
      normalizedPosition = constrain(normalizedPosition, 0.0, 1.0);
    }

    float easedPosition =
        pow(normalizedPosition, 1.0 / BRAKE_EASE_OUT_MULTIPLIER);

    frontServoPosition = BRAKE_SERVO_1_MIN_VALUE +
        (int)(easedPosition *
              (BRAKE_SERVO_1_MAX_VALUE - BRAKE_SERVO_1_MIN_VALUE));
    backServoPosition = BRAKE_SERVO_2_MIN_VALUE +
        (int)(easedPosition *
              (BRAKE_SERVO_2_MAX_VALUE - BRAKE_SERVO_2_MIN_VALUE));
  }

  frontServo.write(frontServoPosition);
  backServo.write(backServoPosition);
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
        digitalWrite(BRAKE_LIGHT_MOSFET_PIN, LOW); // Brake light off
        lightState = false; // Ensure state is consistent
    } 
    else if (brakeLeverPosition <= upperLeverThreshold) 
    {

        if ( speedSensor.isCarStopped() ) 
        {
            // Full brightness: Keep the light on
            digitalWrite(BRAKE_LIGHT_MOSFET_PIN, HIGH);
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
                digitalWrite(BRAKE_LIGHT_MOSFET_PIN, lightState ? HIGH : LOW);
                lastBlinkTime = currentTime; // Reset the timer
            }
        }
    } 
}
/////////////////////////////////////////////////////////////////////
////////////////////          Setup         /////////////////////////
/////////////////////////////////////////////////////////////////////

void BrakeSystem::setup() 
{
  pinMode(BRAKE_LIGHT_MOSFET_PIN, OUTPUT);
  pinMode(BRAKE_LEVER_POT_PIN, INPUT);
  pinMode(HAND_BRAKE_LED_PIN, OUTPUT);
  digitalWrite(BRAKE_LIGHT_MOSFET_PIN, LOW);
  digitalWrite(HAND_BRAKE_LED_PIN, LOW);

  if (ENABLE_HAND_BRAKE_BUTTON)
  {
    pinMode(HAND_BRAKE_BUTTON_PIN, INPUT_PULLUP);
    lastHandBrakeButtonReading = digitalRead(HAND_BRAKE_BUTTON_PIN);
    stableHandBrakeButtonState = lastHandBrakeButtonReading;
  }

  analogReadResolution(12);
  brakeLeverPosition = readBrakeLeverPosition();
  setupServo();
  
  WiFiPrinter::print("BrakeSystem setup complete!");
}

void BrakeSystem::setupServo() 
{
  frontServo.attach(SERVO_BRAKE_FRONT_PIN);
  backServo.attach(SERVO_BRAKE_BACK_PIN);
  frontServo.write(frontServoPosition);
  backServo.write(backServoPosition);
  delay(300);

  update();
}
