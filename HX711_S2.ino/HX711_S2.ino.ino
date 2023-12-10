#include <HX711_ADC.h>
#include <Preferences.h> // Include Preferences library instead of EEPROM

// Define constants for HX711 pins
const int HX711_dout = 2; // Adjusted for S2 Mini GPIO numbers
const int HX711_sck = 4;  // Adjusted for S2 Mini GPIO numbers
const int numReadings = 50;
const float knownWeight = 66.0;
const char *calVal_preferenceKey = "calFactor"; // Key for Preferences

// Initialize HX711
HX711_ADC LoadCell(HX711_dout, HX711_sck);

// Define the state machine states
enum State {
  START_UP,
  IDLE,
  TARE,
  CALIBRATE,
  READ_WEIGHT
} state = START_UP;

// Initialize variables
unsigned long lastReadTime = 0;
const unsigned long readInterval = 10000; // Interval for weight readings
unsigned long startUpStabilizingtime = 2000;

Preferences preferences; // Create a Preferences object

String printStateValue = "";

void printState(String _printStateValue) {
  // Print state if it has changed
  if (printStateValue != _printStateValue) {
    printStateValue = _printStateValue;
    Serial.println(printStateValue);
  }
}

void setup() {
  Serial.begin(9600); // Begin Serial communication at 9600 baud rate
  startUpRoutine(); // Perform initial setup routine
}

void startUpRoutine() {
  // Initialization routine for load cell
  Serial.println("Initial Startup Routine...");
  LoadCell.begin();
  LoadCell.start(startUpStabilizingtime, true);

  // Read calibration value from Preferences
  float calibrationValue = readCalibrationValue();
  LoadCell.setCalFactor(calibrationValue);
  Serial.print("Calibration value: ");
  Serial.println(calibrationValue);

  lastReadTime = millis(); // Record current time
  state = IDLE; // Set state to IDLE after startup
}

void loop() 
{
  // Finite state machine handling
  switch(state) 
  {
    case START_UP:
    {
      printState( "STARTING_UP" );

      state = IDLE;
      break;
    }

    case IDLE:
    {
      printState( "IDLE" );
      // Check for new incoming serial commands
      if (Serial.available() > 0) 
      {
        char inByte = Serial.read();
        if (inByte == 't') 
        {
          state = TARE;
        } 
        else if (inByte == 'r') 
        {
          state = READ_WEIGHT;
        }
        else if (inByte == 'c') 
        {
          state = CALIBRATE;
        }
      }
      break;
    }

    case TARE:
    {

      printState( "TARE" );
      Serial.println("Send 't' from serial monitor to set the tare offset.");
      boolean _resume = false;
      while (_resume == false) 
      {
        LoadCell.update();
        if (Serial.available() > 0) 
        {
          if (Serial.available() > 0) 
          {
            char inByte = Serial.read();
            if (inByte == 't') 
            LoadCell.tareNoDelay();
          }
        }
        if (LoadCell.getTareStatus() == true) 
        {
          Serial.println("Tare complete");
           state = IDLE;
          _resume = true;
        }
      }

      break;

    }

    case CALIBRATE:
    {
      printState( "CALIBRATE" );
      // Perform calibration with known weight

      LoadCell.refreshDataSet(); // Refresh data set for stable readings before calibration
      float newCalibrationValue = LoadCell.getNewCalibration(knownWeight);
      writeCalibrationValue( newCalibrationValue );
      Serial.print("Calibration complete. New calibration value: ");
      Serial.println(newCalibrationValue);
      state = IDLE;
      break;
    }

    case READ_WEIGHT:
    {
      printState( "READ_WEIGHT" );
      // Read weight and average over specified number of readings
      float weight = 0;
      for (int i = 0; i < numReadings; i++) 
      {
        LoadCell.update();
        weight += LoadCell.getData();
        delay(10); // Slight delay between readings
      }

      weight /= numReadings; // Average
      Serial.print("Average weight: ");
      Serial.println(weight);
      lastReadTime = millis();
      state = IDLE;
      break;
    }
  }

  // Also read at regular intervals if not in special modes
  if (state == IDLE && millis() - lastReadTime >= readInterval)
   {
    state = READ_WEIGHT;
  }
}

float readCalibrationValue() {
  // Read calibration value from Preferences
  preferences.begin("calibration", false); // Open Preferences with readonly access
  float calibrationValue = preferences.getFloat(calVal_preferenceKey, 15159.0); // Use default if no value was previously stored
  preferences.end(); // Close Preferences
  Serial.print("Read from Preferences: ");
  Serial.println(calibrationValue);
  return calibrationValue;
}

void writeCalibrationValue(float calibrationValue) {
  // Write calibration value to Preferences
  preferences.begin("calibration", false); // Open Preferences with read/write access
  preferences.putFloat(calVal_preferenceKey, calibrationValue); // Store calibration value using the defined key
  preferences.end(); // Close Preferences
  Serial.print("Written to Preferences with success: ");
  Serial.println(calibrationValue);
}

// The rest of your code remains the same.
// ...