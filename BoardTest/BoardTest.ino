
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"

#include <Arduino.h>
#include <Wire.h>
#include <math.h> // For sine function
#include <Adafruit_ADS1X15.h>
#include <ESP32Servo.h>

//Arduino_GFX setting
#include "Arduino_DataBus.h"
#include "databus/Arduino_ESP32SPI.h"
#include "databus/Arduino_Wire.h"
#include "Arduino_GFX.h"
#include "display/Arduino_GC9A01.h"

#include <DigiPotX9Cxxx.h>
#include "MPU9250.h"


// Create an ADS1115 object
Adafruit_ADS1115 ads_joystick;
bool is_initialized_ADS1115_joystick = false;
bool is_initialized_lcd = false;
bool is_initialized_MPU = false;
unsigned long ADS1115_connection_timeout = 1000; // Total time to attempt connection in milliseconds

Servo servoBrake1; 
Servo servoBrake2; 
Servo servoSteering; 

// Define the digital potentiometer with specified multiplexer channels
DigiPot potentiometer1(16/* nc */, 18 /* ud */ , 8 /* cs */);  
DigiPot potentiometer2(16/* nc */, 18 /* ud */ , 9 /* cs */);

MPU9250 IMU(Wire,0x68); 


// Function to set voltmeter voltage using PWM
void setVoltmeterPWM(int pin, int pwmValue, int channel, int freq = 5000, int resolution = 8) 
{
  // Ensure the PWM value is within the valid range
  if (pwmValue < 0) pwmValue = 0;
  if (pwmValue > 255) pwmValue = 255; // Maximum PWM value for 8-bit resolution

  // Attach the pin to a channel with frequency and resolution
  //ledcAttach(pin, freq, resolution);
  ledcAttachChannel(pin, freq, resolution, channel);

  // Write the PWM value directly to the pin
  ledcWrite(pin, pwmValue);
}


void setup()
{
  Serial.begin(9600);

  initializePins();
  initializeServos();
  initializePotentiometers();
  initializeMPU();

  Serial.println("Board Test App");
}

void initializeServos() 
{
  int servoChannel = servoBrake1.attach(SERVO_BRAKE_1);  // Reattach the servo if it was detached
  Serial.println("servoBrake1 Channel:" + String(servoChannel));

  servoChannel = servoBrake2.attach(SERVO_BRAKE_2);  // Reattach the servo if it was detached
  Serial.println("servoBrake2 Channel:" + String(servoChannel));

  servoChannel = servoSteering.attach(SERVO_STEERING);  // Reattach the servo if it was detached
  Serial.println("servoSteering Channel:" + String(servoChannel));

  delay(100);
}

void initializePotentiometers() 
{
  potentiometer1.set(10);
  potentiometer2.set(90);
}

void initializeMPU() 
{
  int status = IMU.begin();

  if (status < 0) 
  {
    Serial.println("MPU initialization unsuccessful");
    Serial.print("Status: ");
    Serial.println(status);
  }
  else
  {
     Serial.println("MPU initializatied successfully");
     is_initialized_MPU = true;
  }
}

void initializePins() 
{
  // Initialize pins
  pinMode(CAR_KEY_SWITCH, INPUT_PULLUP);
  pinMode(MOSFET_48V, OUTPUT);
  pinMode(VOLTMETER_SPEED, OUTPUT);
  pinMode(VOLTMETER_CHARGING, OUTPUT);
  pinMode(VOLTMETER_BATTERY, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);

  pinMode(DistanceSensor_trigPin, OUTPUT);
  pinMode(DistanceSensor_echoPin, INPUT);

  Serial.println("Pins initialize");
}

void readSerialInput() 
{
  if (Serial.available() > 0) 
  {
    String input = Serial.readStringUntil('\n');  // Read the input from serial monitor
    int targetPMW = input.toInt();  // Get the target angle

    if (targetPMW < 0) targetPMW = 0;
    if (targetPMW > 255) targetPMW = 255; // Maximum PWM value for 8-bit resolution

    Serial.println("servo:"+ input);
    servoBrake1.write(targetPMW);  // Move the servo to the target angle
    servoBrake2.write(targetPMW);  // Move the servo to the target angle
    servoSteering.write(targetPMW);  // Move the servo to the target angle


    setVoltmeterPWM(VOLTMETER_SPEED,    targetPMW,  5);
    setVoltmeterPWM(VOLTMETER_CHARGING, targetPMW,  6);
    setVoltmeterPWM(VOLTMETER_BATTERY,  targetPMW,  7);

    int potValue = map(targetPMW, 0, 255, 0, 100);
    potentiometer1.set(potValue);
    potentiometer2.set(potValue);
  }
}

void loop()
{
  handleCarKeySwitch();
  updateVoltmeters();
  //updateServos();
  //updatePotentiometers();
  //updateMPU();
  updateDistanceSensor();
}




void updateServos()
{
  if(is_initialized_ADS1115_joystick)
  {
    servoBrake1.write( readJoystick( 0 ) );
    servoBrake2.write( readJoystick( 1 ) );
    servoSteering.write( readJoystick( 2 ) );
  }
  else
  {
    readSerialInput();  // Call the method to read serial input
  }
}

void updatePotentiometers()
{
  if( !is_initialized_ADS1115_joystick )
    return;

  int potValue1 = map(readJoystick( 0 ), 0, 255, 0, 100);
  int potValue2 = map(readJoystick( 1 ), 0, 255, 0, 100);

  potentiometer1.set(potValue1);
  potentiometer2.set(potValue2);
}

// Define the necessary variables
unsigned long MPU_LastUpdateTime = 0; // Stores the last update time
unsigned long MPU_UpdateInterval = 50; // Set the interval in milliseconds (can be updated as needed)

void updateMPU()
{
  unsigned long currentTime = millis();

  // Check if the required time interval has passed
  if (currentTime - MPU_LastUpdateTime < MPU_UpdateInterval)
  {
    return; // Exit the method if the interval hasn't passed
  }

  // Update the last update time
  MPU_LastUpdateTime = currentTime;

  // Read sensor data
  IMU.readSensor();

  int carAngle =  map(IMU.getAccelZ_mss() * 1000, -10000, 10000, 0, 255);

  // Display the data
  Serial.print(carAngle, 6);
  Serial.println("\t");

  setVoltmeterPWM(VOLTMETER_BATTERY, carAngle, 7);
}

unsigned long previousMillis = 0;  // Will store the last time the distance sensor was updated
const long DistanceSensor_UpdateInterval = 1000;  // Interval between updates in milliseconds
long duration;
int distance;
void updateDistanceSensor()
{
  unsigned long currentMillis = millis();  // Get current time in milliseconds

  if (currentMillis - previousMillis >= DistanceSensor_UpdateInterval)
   {
    // Save the last time the sensor was updated
    previousMillis = currentMillis;

    // Trigger the sensor
    digitalWrite(DistanceSensor_trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(DistanceSensor_trigPin, HIGH);
    delayMicroseconds(20);
    digitalWrite(DistanceSensor_trigPin, LOW);

    // Measure the pulse width
    duration = pulseIn(DistanceSensor_echoPin, HIGH);

    // Calculate the distance in cm
    distance = duration * 0.034 / 2.0;

    // Print the distance (or do something with it)
    Serial.println(distance);
  }
}


void updateVoltmeters()
{
  if( !is_initialized_ADS1115_joystick )
    return;

  setVoltmeterPWM(VOLTMETER_SPEED,    readJoystick( 2 ),  5);
  setVoltmeterPWM(VOLTMETER_CHARGING, readJoystick( 1 ),  6);
  //setVoltmeterPWM(VOLTMETER_BATTERY,  readJoystick( 0 ),  7);
}
  
// Function to handle carKeySwitch 
void handleCarKeySwitch()
{
  // Declare a static variable to store the previous state of the car key switch
  static int previousCarKeySwitchState = LOW;

  // Read the current state of the car key switch
  int carKeySwitchState = digitalRead(CAR_KEY_SWITCH);


  // Check if the state has changed
  if (carKeySwitchState != previousCarKeySwitchState) 
  {
    if (carKeySwitchState == LOW) 
      turnOnCar();
    else 
      turnOffCar();

    // Update the previous state
    previousCarKeySwitchState = carKeySwitchState;
  }
}





void turnOnCar()
{
  Serial.println("Turning Car On");

  digitalWrite(MOSFET_48V, HIGH); 
 
  unsigned long ADS1115_connection_time_Start = millis(); // Record the time when the connection attempt starts

  while (millis() - ADS1115_connection_time_Start < ADS1115_connection_timeout)
  {
    if (ads_joystick.begin(0x4A)) 
    {
        is_initialized_ADS1115_joystick = true;
        ads_joystick.setGain(GAIN_ONE);

         Serial.println("ADS1115 inilialized");

        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }

  // If initialization failed after the timeout, call the error handling method
  if (!is_initialized_ADS1115_joystick) 
  {
    registerError( "Could not inilialize ADS1115 for joystick" );
  }
  

  
   digitalWrite(LED_BUILTIN, HIGH);

   Serial.println("Car Turned On");
}



void turnOffCar()
{
  Serial.println("Turning Car Off");

  setVoltmeterPWM(VOLTMETER_SPEED,     0,  5);
  setVoltmeterPWM(VOLTMETER_CHARGING, 0,  6);
  setVoltmeterPWM(VOLTMETER_BATTERY,  0,  7);
  

  digitalWrite(LED_BUILTIN, LOW);
  digitalWrite(MOSFET_48V, LOW); 

  is_initialized_ADS1115_joystick = false;
  is_initialized_lcd = false;

  Serial.println("Car Turned Off");
}

// Function to read joystick value from ADS1115 A2 pin and map to PWM
int readJoystick( int adcPin ) 
{
  // Read ADC value (16-bit signed integer, -32768 to 32767)
  int16_t rawValue = ads_joystick.readADC_SingleEnded( adcPin );

  // Convert raw ADC value to PWM range (0-255)
  // ADS1115 range: 0-32767 maps to 0-3.3V
  int pwmValue = map(rawValue, 0, 32767, 0, PWM_RESOLUTION);

  // Ensure PWM value is within the valid range
  pwmValue = constrain(pwmValue, 0, PWM_RESOLUTION);

  return pwmValue;
}




void registerError(const char* errorMessage)
{
    Serial.println(errorMessage); // Print the error message to the Serial Monitor
}
