#include "Dashboard.h"
#include "WiFiPrinter.h"
#include "PinDefinitions.h"
#include "ConstantDefinitions.h"
#include "PortExpander.h"
#include "Buzzer.h"
#include "Horn.h"

// Constructor
Dashboard::Dashboard(VoltageSensor& sensorInstance)
  : voltageSensor(sensorInstance)
{

}

// Initialize the Dashboard
void Dashboard::start() 
{
  pinMode(VOLTMETER_SPEED, OUTPUT);
  pinMode(VOLTMETER_CHARGING, OUTPUT);
  pinMode(VOLTMETER_BATTERY, OUTPUT);


  buttonState[0] = HIGH;
  buttonState[1] = HIGH;
  buttonState[2] = HIGH;
  buttonState[3] = HIGH;

  toggleState[0] = LOW;
  toggleState[1] = LOW;
  toggleState[2] = LOW;
  toggleState[3] = LOW;

  unsigned long module_connection_time_Start = millis(); // Record the time when the connection attempt starts

  while (millis() - module_connection_time_Start < MODULE_CONNECTION_TIMEOUT)
  {
    if (ads1115.begin(0x4A)) 
    {
        initialized = true;
        ads1115.setGain(GAIN_ONE);

        // Calculate how long it took to initialize in ms
        unsigned long initialization_time = millis() - module_connection_time_Start;

        // Print the initialization time
        WiFiPrinter::print("ADS1115 joystick initialized in " + String( initialization_time ) + "ms");

        break; // Exit the loop if initialization is successful
    }
    delay(10); // Wait 100 ms before retrying
  }

  // If initialization failed after the timeout, call the error handling method
  if (!initialized) 
  {
     WiFiPrinter::print( "Could not inilialize ADS1115 for joystick" );
     Buzzer::getInstance().beep3();
  }
}

// Deinitialize the Dashboard
void Dashboard::shutdown() 
{
  digitalWrite(VOLTMETER_SPEED, LOW);
  digitalWrite(VOLTMETER_CHARGING, LOW);
  digitalWrite(VOLTMETER_BATTERY, LOW);

  // Set GPIOs as INPUT to avoid any backfeeding
  pinMode(VOLTMETER_SPEED, INPUT);
  pinMode(VOLTMETER_CHARGING, INPUT);
  pinMode(VOLTMETER_BATTERY, INPUT);

  // Set I2C lines to INPUT to release the bus
  pinMode(SDA, INPUT);
  pinMode(SCL, INPUT);

  // Delay to ensure shutdown stabilization
  delay(10);

  initialized = false;
}

// Update the Dashboard (main logic)
void Dashboard::update() 
{
  if( !initialized)
    return;

  updateButtons();
  updateJoysticks();
  updateVoltmeters();
}

bool Dashboard::hasBraked()
{
  return (joystick_throttle < JOYSTICK_THROTTLE_REST_MIN);
}

// Update button states
void Dashboard::updateButtons()
{
  PortExpander& portExpander = PortExpander::getInstance();

  if( !portExpander.initialized )
    return;

  static unsigned long lastLOWTime[4] = {0, 0, 0, 0};
  static unsigned long provisionalButtonState[4] = {HIGH, HIGH, HIGH, HIGH};

  uint8_t currentButtonState;

  static bool  updateToggleState[4] = {false, false, false, false};
  
  PortExpanderPin buttons[] = 
  {
    BUTTON_0_PIN, 
    BUTTON_1_PIN, 
    BUTTON_2_PIN, 
    BUTTON_3_PIN
  };

  for (uint8_t i = 0; i < 4; i++)
  {
    currentButtonState = portExpander.digitalReadMCP23S17( buttons[i] );

    if (currentButtonState != provisionalButtonState[i]) 
    {
      if (currentButtonState == HIGH) 
      {
        buttonState[i] = HIGH;
        provisionalButtonState[i] = HIGH;
      }
      else 
      {
        lastLOWTime[i] = millis();
        provisionalButtonState[i] = LOW;
      }
    }

    if (currentButtonState == LOW && provisionalButtonState[i] == LOW &&  buttonState[i] == HIGH) 
    {
      if( millis() - lastLOWTime[i] > DASHBOARD_BUTTON_DEBOUNCE_DELAY )
      {
        buttonState[i] = LOW;
        toggleState[i] = !toggleState[i];
        updateToggleState[i] = true;
        Buzzer::getInstance().beep();

        if (i == 1)
        {
          Horn::getInstance().beep();
        }
        else if( i == 3)
        {
          requestWiFiCallback();
        }
      }
    }
  }

  if( updateToggleState[0] )
  {
    portExpander.digitalWriteMCP23S17(MOSFET_NIGH_LIGHT_PIN, toggleState[0]);
    updateToggleState[0]= false;
  }

  if( updateToggleState[1] )
  {
    updateToggleState[1]= false;
  }

  if( updateToggleState[2] )
  {
    updateToggleState[2]= false;
  }

  if( updateToggleState[3] )
  {
    updateToggleState[3]= false;
  }

  

  if( buttonState[0] == LOW &&  buttonState[1] == LOW && buttonState[2] == HIGH &&  buttonState[3] == HIGH)
  {
    //requestWiFiCallback();
  }

}

void Dashboard::onRequestWiFi(std::function<void()> callback) {
    requestWiFiCallback = callback;
}

// Update joystick states
void Dashboard::updateJoysticks()
{
  unsigned long currentTime = millis();  // Get the current time

  if (currentTime - joystickLastUpdateTime >= MPU_UPDATE_INTERVAL) 
  {  // Check if enough time has passed
    joystick_steering = readJoystick(3);
    joystick_throttle = constrain(readJoystick(0), 0, JOYSTICK_THROTTLE_MAX_VALUE);
    joystick_knob =     constrain(readJoystick(1), 0, JOYSTICK_THROTTLE_MAX_VALUE);
    
    joystickLastUpdateTime = currentTime;  // Update the last update time
  }
}

// Update voltmeter readings
void Dashboard::updateVoltmeters()
{
  unsigned long currentTime = millis();  // Get the current time

  // Check if enough time has passed
  if (currentTime - voltMetersLastUpdateTime >= VOLTMETER_UPDATE_INTERVAL) 
  {  
    
    //map the voltage percentage
    int pwmVoltagePercentageValue = map(voltageSensor.batteryPercentage, 0, 100, 0, 255);// Map percentage to PWM range (0-255)

    //setVoltmeterPWM(VOLTMETER_BATTERY,  pwmVoltagePercentageValue,  VOLTMETER_CHARGING_CHANNEL);
    setVoltmeterPWM(VOLTMETER_SPEED,      joystick_steering,          VOLTMETER_SPEED_CHANNEL);
    setVoltmeterPWM(VOLTMETER_CHARGING,   joystick_knob,              VOLTMETER_BATTERY_CHANNEL);
    setVoltmeterPWM(VOLTMETER_BATTERY,    pwmVoltagePercentageValue,  VOLTMETER_CHARGING_CHANNEL);

    voltMetersLastUpdateTime = currentTime;  // Update the last update time
  }
}

// Set PWM for a voltmeter
void Dashboard::setVoltmeterPWM(int pin, int pwmValue, int channel, int freq, int resolution)
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

// Function to read joystick value from ADS1115 A2 pin and map to PWM
int Dashboard::readJoystick(int adcPin)
{
  // Read ADC value (16-bit signed integer, -32768 to 32767)
  int16_t rawValue = ads1115.readADC_SingleEnded( adcPin );

  // Convert raw ADC value to PWM range (0-255)
  // ADS1115 range: 0-32767 maps to 0-3.3V
  int pwmValue = map(rawValue, 0, 32767, 0, PWM_RESOLUTION);

  // Ensure PWM value is within the valid range
  pwmValue = constrain(pwmValue, 0, PWM_RESOLUTION);

  return pwmValue;
}

