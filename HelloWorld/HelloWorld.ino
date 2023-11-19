#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>


#define S0 D0                             /* Assign Multiplexer pin S0 connect to pin D0 of NodeMCU */
#define S1 D1                             /* Assign Multiplexer pin S1 connect to pin D1 of NodeMCU */
#define S2 D2                             /* Assign Multiplexer pin S2 connect to pin D2 of NodeMCU */
#define S3 D3                             /* Assign Multiplexer pin S3 connect to pin D3 of NodeMCU */
#define SIG A0                            /* Assign SIG pin as Analog output for all 16 channels of Multiplexer to pin A0 of NodeMCU */

int joystick_x;
int joystick_y;

int pixel_x = 0;
int pixel_y = 0;

bool  joystick_button;


U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

void setup(void) 
{
  u8g2.begin();

  pinMode(S0, OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin SO */        
  pinMode(S1, OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin S1 */  
  pinMode(S2, OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin S2 */ 
  pinMode(S3, OUTPUT);                       /* Define digital signal pin as output to the Multiplexer pin S3 */  
  pinMode(SIG, INPUT);                      /* Define analog signal pin as input or receiver from the Multiplexer pin SIG */
}

void loop(void)
{
  joystick_x = readJoystickX();
  joystick_y = readJoystickY();
  joystick_button = readJoystickButton();

  //u8g2.clearBuffer();					       // clear the internal memory
  //u8g2.setFont(u8g2_font_7x14B_tr);	 // choose a suitable font

  //if( joystick_button )
  //  u8g2.drawStr(30,10,"Exupéry V2!");

  // clear drawn pixel only
  u8g2.setDrawColor(0);
  u8g2.drawPixel( pixel_x, pixel_y);

  pixel_x =  (joystick_x * 127) / 1024;
  pixel_y = 63 - ((joystick_y * 63) / 1024);

  u8g2.setDrawColor(1);
  u8g2.drawPixel( pixel_x, pixel_y);

  u8g2.sendBuffer();					       // transfer internal memory to the display
}

int readJoystickX()
{
  // Channel 0 (C0 pin - binary output 0,0,0,0)
  digitalWrite(S0,LOW); digitalWrite(S1,LOW); digitalWrite(S2,LOW); digitalWrite(S3,LOW);
  return analogRead(SIG);
}

int readJoystickY()
{
  // Channel 1 (C1 pin - binary output 1,0,0,0)
  digitalWrite(S0,HIGH); digitalWrite(S1,LOW); digitalWrite(S2,LOW); digitalWrite(S3,LOW);
  return analogRead(SIG);
}

bool  readJoystickButton()
{
  // Channel 1 (C1 pin - binary output 1,0,0,0)
  digitalWrite(S0,LOW); digitalWrite(S1,HIGH); digitalWrite(S2,LOW); digitalWrite(S3,LOW);
  if ( analogRead(SIG) > 30)
    return false;
  else
    return true;
}



