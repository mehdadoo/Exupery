#include <HX711_ADC.h>
#include <EEPROM.h>

const int HX711_dout = D2; // HX711 dout pin
const int HX711_sck = D3;  // HX711 sck pin
const int numReadings = 50; // Number of readings to smooth
const float knownWeight = 66.0; // Known weight for calibration
const int calVal_eepromAddress = 0; // EEPROM address for storing the calibration factor

HX711_ADC LoadCell(HX711_dout, HX711_sck);

enum State 
{
  START_UP,
  IDLE,
  TARE,
  CALIBRATE,
  READ_WEIGHT
} state = START_UP;

unsigned long lastReadTime = 0;
const unsigned long readInterval = 10000; // Read weight every 10 seconds
unsigned long startUpStabilizingtime = 2000;

String printStateValue = "";
void printState(String _printStateValue)
{
  if (printStateValue != _printStateValue) 
  {
    printStateValue = _printStateValue;
    Serial.println( printStateValue );
  }
}

void setup() 
{
  Serial.begin(9600); 
  startUpRoutine();
  
}



void startUpRoutine()
{
  Serial.println("Initial Startup Routine...");
  
  LoadCell.begin();
  LoadCell.start(startUpStabilizingtime, true);

  float calibrationValue;
  readCalibrationValue(&calibrationValue);
  LoadCell.setCalFactor(calibrationValue);
  Serial.println( calibrationValue );

  Serial.print("Calibration complete. New calibration value: ");
  Serial.println(calibrationValue);

  lastReadTime = millis();
  state = START_UP;
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

bool readCalibrationValue(float *calibrationValue) {
  // Create a temporary buffer to hold the read bytes from EEPROM
  union {
    float calibrationValue;
    byte bytes[sizeof(float)];
  } data;

  // Read the bytes from EEPROM into the buffer
  for (unsigned int i = 0; i < sizeof(float); i++) {
    data.bytes[i] = EEPROM.read(calVal_eepromAddress + i);
  }

  *calibrationValue = data.calibrationValue;

  Serial.print("read from EEPROM: ");
  Serial.println(*calibrationValue);

  // Check if the calibration value read makes sense
  if (*calibrationValue >= 1e-6 && *calibrationValue <= 1e6) 
  {
    
    return true; // Valid calibration value was read
  }

  

  // If the read value was not valid, assign a default value
  *calibrationValue = 15159.0; // Default value if not valid or not set

  Serial.print("returned from EEPROM: ");
  Serial.println(*calibrationValue);
  return false; // Indicate that we're returning a default value
}

void writeCalibrationValue(float calibrationValue) {
  // Create a buffer to hold the bytes to be written to EEPROM
  union {
    float calibrationValue;
    byte bytes[sizeof(float)];
  } data;

  data.calibrationValue = calibrationValue;

  // Write the bytes to EEPROM
  for (unsigned int i = 0; i < sizeof(float); i++) {
    EEPROM.write(calVal_eepromAddress + i, data.bytes[i]);
  }

  EEPROM.commit(); // Ensure EEPROM write is completed properly

  Serial.print("Written to EEPROM with success: ");
  Serial.println(calibrationValue);
}