#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <Servo.h>


#define S0 D0                             /* Assign Multiplexer pin S0 connect to pin D0 of NodeMCU */
#define S1 D1                             /* Assign Multiplexer pin S1 connect to pin D1 of NodeMCU */
#define S2 D2                             /* Assign Multiplexer pin S2 connect to pin D2 of NodeMCU */
#define SIG A0                            /* Assign SIG pin as Analog output for all 16 channels of Multiplexer to pin A0 of NodeMCU */


//#define JoystickButtonPin D7 
//#define Button1Pin D3 
//#define Button2Pin D4 
//#define Button3Pin D8 
      

int buttonJoystick = 0;
int buttonState1 = 0;
int buttonState2 = 0;
int buttonState3 = 0;

int joystick_x;
int joystick_y;

int pixel_x = 0;
int pixel_y = 0;

U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

void setup(void) 
{
  Serial.begin(9600);
  u8g2.begin();


  pinMode(S0, OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin SO */        
  pinMode(S1, OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin S1 */  
  pinMode(S2, OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin S2 */ 
  pinMode(SIG, INPUT);                      /* Define analog signal pin as input or receiver from the Multiplexer pin SIG */

  //pinMode(Button1Pin, INPUT);
  //pinMode(Button2Pin, INPUT);
  //pinMode(JoystickButtonPin, INPUT_PULLUP);
  //pinMode(Button3Pin, INPUT);
}

void loop(void)
{
  joystick_x = readMultiplexer(LOW, LOW, LOW);
  joystick_y = readMultiplexer(HIGH, LOW, LOW);

  buttonJoystick = readMultiplexer(LOW, HIGH, LOW);
  buttonState1 = readMultiplexer(HIGH, HIGH, LOW);
  buttonState2 = readMultiplexer(LOW, LOW, HIGH);
  buttonState3 = readMultiplexer(HIGH, LOW, HIGH);

  Serial.print( buttonState1 );
  Serial.print( ", " );
  Serial.print( buttonState2 );
  Serial.print( ", " );
  Serial.print( buttonState3 );
  Serial.print( ", " );
  Serial.print( buttonJoystick );
  Serial.println();

  u8g2.clearBuffer();			


  // u8g2.setFont(u8g2_font_7x14B_tr);	 // choose a suitable font
  //  u8g2.drawStr(30,10,"Exupéry V2!");

  // clear drawn pixel only
  // u8g2.setDrawColor(0);
  // u8g2.drawPixel( pixel_x, pixel_y);

  pixel_x =   ((joystick_x * 44) / 1024) + 1;
  pixel_y = 62 - ((joystick_y * 44) / 1024);

  u8g2.setDrawColor(1);
  u8g2.drawPixel( pixel_x, pixel_y);

  u8g2.drawFrame(0,16,48,48);
  
  /*Serial.print( !digitalRead(Button1Pin) );
  Serial.print(", ");
  Serial.print( !digitalRead(Button2Pin) );
  Serial.print(", ");
  Serial.print( digitalRead(Button3Pin) );
  Serial.print(", ");
  Serial.print( !digitalRead(JoystickButtonPin) );
  Serial.println();


  if ( !digitalRead(Button1Pin) ) 
  {
      u8g2.drawFrame(0,0,16,16);
  } 

  if ( !digitalRead(Button2Pin) ) 
  {
      u8g2.drawFrame(16,0,16,16);
  } 

  if ( digitalRead(Button3Pin) ) 
  {
      u8g2.drawFrame(32,0,16,16);
  } 

   if ( !digitalRead(JoystickButtonPin) ) 
  {
      u8g2.drawFrame(48,0,16,16);
  } 
*/


  u8g2.sendBuffer();	  // transfer internal memory to the display
}

int readMultiplexer( boolean _s0, boolean  _s1, boolean  _s2)
{
  digitalWrite(S0,_s0); digitalWrite(S1,_s1); digitalWrite(S2,_s2); 
  return analogRead(SIG);
}




