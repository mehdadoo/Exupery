
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

Servo servoBrake1; 
Servo servoBrake2; 
Servo servoSteering; 

// Define the digital potentiometer with specified multiplexer channels
DigiPot potentiometer1(DigiPot_NC, DigiPot_UD , DigiPot_CS1);  
DigiPot potentiometer2(DigiPot_NC, DigiPot_UD , DigiPot_CS2);


MPU9250 IMU(Wire,0x68); 

//Arduino_GFX setting
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_SCK, TFT_MOSI, GFX_NOT_DEFINED, HSPI /* spi_num */);
Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

//display variables
static int16_t w, h, center;
static int16_t hHandLen, mHandLen, sHandLen, markLen;
static float sdeg, mdeg, hdeg;
static int16_t osx = 0, osy = 0, omx = 0, omy = 0, ohx = 0, ohy = 0; // Saved H, M, S x & y coords
static int16_t nsx, nsy, nmx, nmy, nhx, nhy;                         // H, M, S x & y coords
static int16_t xMin, yMin, xMax, yMax;                               // redraw range
static int16_t hh, mm, ss;
static unsigned long targetTime; // next action time

static int16_t *cached_points;
static uint16_t cached_points_idx = 0;
static int16_t *last_cached_point;



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

  delay(50);
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
    Serial.println("MPU initialization unsuccessful!!!!!!!!");
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
  //updateVoltmeters();
  updateServos();
  updatePotentiometers();
  updateMPU();
  updateDistanceSensor();
  updateLCD();
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

void updateMPU()
{
  unsigned long currentTime = millis();

  // Check if the required time interval has passed
  if (currentTime - MPU_LastUpdateTime < MPU_UPDATE_INTERVAL)
  {
    return; // Exit the method if the interval hasn't passed
  }

  // Update the last update time
  MPU_LastUpdateTime = currentTime;

  // Read sensor data
  IMU.readSensor();

  int carAngle =  map(IMU.getAccelZ_mss() * 1000, -10000, 10000, 0, 255);

  // Display the data
  Serial.print("MPU:");
  Serial.print(carAngle, 6);
  Serial.println("\t");

}

// Define the necessary variables
unsigned long DistanceSensor_LastUpdateTime = 0; // Stores the last update time
long duration;
int distance;
void updateDistanceSensor()
{
  unsigned long currentTime = millis();

  // Check if the required time interval has passed
  if (currentTime - DistanceSensor_LastUpdateTime < MPU_UPDATE_INTERVAL)
  {
    return; // Exit the method if the interval hasn't passed
  }

  // Update the last update time
  DistanceSensor_LastUpdateTime = currentTime;


  // Trigger the sensor
  digitalWrite(DistanceSensor_trigPin, LOW);
  delayMicroseconds(1);
  digitalWrite(DistanceSensor_trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(DistanceSensor_trigPin, LOW);

  // Measure the pulse width
  duration = pulseIn(DistanceSensor_echoPin, HIGH);

  // Calculate the distance in cm
  distance = duration * 0.034 / 2.0;

  // Print the distance (or do something with it)
  Serial.print("Distance:");
  Serial.print(distance);
  Serial.println("\t");
}


void updateVoltmeters()
{
  if( !is_initialized_ADS1115_joystick )
    return;

  setVoltmeterPWM(VOLTMETER_SPEED,    readJoystick( 2 ),  5);
  setVoltmeterPWM(VOLTMETER_CHARGING, readJoystick( 1 ),  6);
  setVoltmeterPWM(VOLTMETER_BATTERY,  readJoystick( 0 ),  7);
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

  while (millis() - ADS1115_connection_time_Start < ADS1115_CONNECTION_TIMEOUT)
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
  
  setupLCD();
  
  digitalWrite(LED_BUILTIN, LOW);

  Serial.println("Car Turned On");
}



void turnOffCar()
{
  Serial.println("Turning Car Off");

  if(is_initialized_lcd)
    gfx->fillScreen(BACKGROUND);

  setVoltmeterPWM(VOLTMETER_SPEED,     0,  5);
  setVoltmeterPWM(VOLTMETER_CHARGING, 0,  6);
  setVoltmeterPWM(VOLTMETER_BATTERY,  0,  7);
  

  digitalWrite(LED_BUILTIN, HIGH);
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




void setupLCD()
{
  Serial.println("setting up LCD");

  #ifdef GFX_EXTRA_PRE_INIT
    GFX_EXTRA_PRE_INIT();
  #endif
  
  // Init Display
  if (!gfx->begin())
  {
    Serial.println("gfx->begin() failed!");
    is_initialized_lcd = false;
    return;
  }
  


  is_initialized_lcd = true;

  
  gfx->fillScreen(BACKGROUND);


  // init LCD constant
  w = gfx->width();
  h = gfx->height();
  if (w < h)
  {
    center = w / 2;
  }
  else
  {
    center = h / 2;
  }
  hHandLen = center * 3 / 8;
  mHandLen = center * 2 / 3;
  sHandLen = center * 5 / 6;
  markLen = sHandLen / 6;
  cached_points = (int16_t *)malloc((hHandLen + 1 + mHandLen + 1 + sHandLen + 1) * 2 * 2);

  // Draw 60 clock marks
  draw_round_clock_mark(
      // draw_square_clock_mark(
      center - markLen, center,
      center - (markLen * 2 / 3), center,
      center - (markLen / 2), center);

  hh = conv2d(__TIME__);
  mm = conv2d(__TIME__ + 3);
  ss = conv2d(__TIME__ + 6);

  targetTime = ((millis() / 1000) + 1) * 1000;
}

void updateLCD()
{
  if( !is_initialized_lcd )
    return;


  unsigned long cur_millis = millis();
  if (cur_millis >= targetTime)
  {
    targetTime += 1000;
    ss++; // Advance second
    if (ss == 60)
    {
      ss = 0;
      mm++; // Advance minute
      if (mm > 59)
      {
        mm = 0;
        hh++; // Advance hour
        if (hh > 23)
        {
          hh = 0;
        }
      }
    }
  }

  // Pre-compute hand degrees, x & y coords for a fast screen update
  sdeg = SIXTIETH_RADIAN * ((0.001 * (cur_millis % 1000)) + ss); // 0-59 (includes millis)
  nsx = cos(sdeg - RIGHT_ANGLE_RADIAN) * sHandLen + center;
  nsy = sin(sdeg - RIGHT_ANGLE_RADIAN) * sHandLen + center;
  if ((nsx != osx) || (nsy != osy))
  {
    mdeg = (SIXTIETH * sdeg) + (SIXTIETH_RADIAN * mm); // 0-59 (includes seconds)
    hdeg = (TWELFTH * mdeg) + (TWELFTH_RADIAN * hh);   // 0-11 (includes minutes)
    mdeg -= RIGHT_ANGLE_RADIAN;
    hdeg -= RIGHT_ANGLE_RADIAN;
    nmx = cos(mdeg) * mHandLen + center;
    nmy = sin(mdeg) * mHandLen + center;
    nhx = cos(hdeg) * hHandLen + center;
    nhy = sin(hdeg) * hHandLen + center;

    // redraw hands
    redraw_hands_cached_draw_and_erase();

    ohx = nhx;
    ohy = nhy;
    omx = nmx;
    omy = nmy;
    osx = nsx;
    osy = nsy;

  }
}

static uint8_t conv2d(const char *p)
{
  uint8_t v = 0;
  return (10 * (*p - '0')) + (*++p - '0');
}

void draw_round_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3)
{
  float x, y;
  int16_t x0, x1, y0, y1, innerR, outerR;
  uint16_t c;

  for (uint8_t i = 0; i < 60; i++)
  {
    if ((i % 15) == 0)
    {
      innerR = innerR1;
      outerR = outerR1;
      c = MARK_COLOR;
    }
    else if ((i % 5) == 0)
    {
      innerR = innerR2;
      outerR = outerR2;
      c = MARK_COLOR;
    }
    else
    {
      innerR = innerR3;
      outerR = outerR3;
      c = SUBMARK_COLOR;
    }

    mdeg = (SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN;
    x = cos(mdeg);
    y = sin(mdeg);
    x0 = x * outerR + center;
    y0 = y * outerR + center;
    x1 = x * innerR + center;
    y1 = y * innerR + center;

    gfx->drawLine(x0, y0, x1, y1, c);
  }
}

void draw_square_clock_mark(int16_t innerR1, int16_t outerR1, int16_t innerR2, int16_t outerR2, int16_t innerR3, int16_t outerR3)
{
  float x, y;
  int16_t x0, x1, y0, y1, innerR, outerR;
  uint16_t c;

  for (uint8_t i = 0; i < 60; i++)
  {
    if ((i % 15) == 0)
    {
      innerR = innerR1;
      outerR = outerR1;
      c = MARK_COLOR;
    }
    else if ((i % 5) == 0)
    {
      innerR = innerR2;
      outerR = outerR2;
      c = MARK_COLOR;
    }
    else
    {
      innerR = innerR3;
      outerR = outerR3;
      c = SUBMARK_COLOR;
    }

    if ((i >= 53) || (i < 8))
    {
      x = tan(SIXTIETH_RADIAN * i);
      x0 = center + (x * outerR);
      y0 = center + (1 - outerR);
      x1 = center + (x * innerR);
      y1 = center + (1 - innerR);
    }
    else if (i < 23)
    {
      y = tan((SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN);
      x0 = center + (outerR);
      y0 = center + (y * outerR);
      x1 = center + (innerR);
      y1 = center + (y * innerR);
    }
    else if (i < 38)
    {
      x = tan(SIXTIETH_RADIAN * i);
      x0 = center - (x * outerR);
      y0 = center + (outerR);
      x1 = center - (x * innerR);
      y1 = center + (innerR);
    }
    else if (i < 53)
    {
      y = tan((SIXTIETH_RADIAN * i) - RIGHT_ANGLE_RADIAN);
      x0 = center + (1 - outerR);
      y0 = center - (y * outerR);
      x1 = center + (1 - innerR);
      y1 = center - (y * innerR);
    }
    gfx->drawLine(x0, y0, x1, y1, c);
  }
}

void redraw_hands_cached_draw_and_erase()
{
  gfx->startWrite();
  draw_and_erase_cached_line(center, center, nsx, nsy, SECOND_COLOR, cached_points, sHandLen + 1, false, false);
  draw_and_erase_cached_line(center, center, nhx, nhy, HOUR_COLOR, cached_points + ((sHandLen + 1) * 2), hHandLen + 1, true, false);
  draw_and_erase_cached_line(center, center, nmx, nmy, MINUTE_COLOR, cached_points + ((sHandLen + 1 + hHandLen + 1) * 2), mHandLen + 1, true, true);
  gfx->endWrite();
}

void draw_and_erase_cached_line(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t color, int16_t *cache, int16_t cache_len, bool cross_check_second, bool cross_check_hour)
{
#if defined(ESP8266)
  yield();
#endif
  bool steep = _diff(y1, y0) > _diff(x1, x0);
  if (steep)
  {
    _swap_int16_t(x0, y0);
    _swap_int16_t(x1, y1);
  }

  int16_t dx, dy;
  dx = _diff(x1, x0);
  dy = _diff(y1, y0);

  int16_t err = dx / 2;
  int8_t xstep = (x0 < x1) ? 1 : -1;
  int8_t ystep = (y0 < y1) ? 1 : -1;
  x1 += xstep;
  int16_t x, y, ox, oy;
  for (uint16_t i = 0; i <= dx; i++)
  {
    if (steep)
    {
      x = y0;
      y = x0;
    }
    else
    {
      x = x0;
      y = y0;
    }
    ox = *(cache + (i * 2));
    oy = *(cache + (i * 2) + 1);
    if ((x == ox) && (y == oy))
    {
      if (cross_check_second || cross_check_hour)
      {
        write_cache_pixel(x, y, color, cross_check_second, cross_check_hour);
      }
    }
    else
    {
      write_cache_pixel(x, y, color, cross_check_second, cross_check_hour);
      if ((ox > 0) || (oy > 0))
      {
        write_cache_pixel(ox, oy, BACKGROUND, cross_check_second, cross_check_hour);
      }
      *(cache + (i * 2)) = x;
      *(cache + (i * 2) + 1) = y;
    }
    if (err < dy)
    {
      y0 += ystep;
      err += dx;
    }
    err -= dy;
    x0 += xstep;
  }
  for (uint16_t i = dx + 1; i < cache_len; i++)
  {
    ox = *(cache + (i * 2));
    oy = *(cache + (i * 2) + 1);
    if ((ox > 0) || (oy > 0))
    {
      write_cache_pixel(ox, oy, BACKGROUND, cross_check_second, cross_check_hour);
    }
    *(cache + (i * 2)) = 0;
    *(cache + (i * 2) + 1) = 0;
  }
}

void write_cache_pixel(int16_t x, int16_t y, int16_t color, bool cross_check_second, bool cross_check_hour)
{
  int16_t *cache = cached_points;
  if (cross_check_second)
  {
    for (uint16_t i = 0; i <= sHandLen; i++)
    {
      if ((x == *(cache++)) && (y == *(cache)))
      {
        return;
      }
      cache++;
    }
  }
  if (cross_check_hour)
  {
    cache = cached_points + ((sHandLen + 1) * 2);
    for (uint16_t i = 0; i <= hHandLen; i++)
    {
      if ((x == *(cache++)) && (y == *(cache)))
      {
        return;
      }
      cache++;
    }
  }
  gfx->writePixel(x, y, color);
}