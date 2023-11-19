/***************************************************************************
* Example sketch for the ADS1115_WE library
*
* This sketch shows how to use the ADS1115 in single shot mode. 
*  
* Further information can be found on:
* https://wolles-elektronikkiste.de/ads1115 (German)
* https://wolles-elektronikkiste.de/en/ads1115-a-d-converter-with-amplifier (English)
* 
***************************************************************************/

#include<ADS1115_WE.h> 
#include<Wire.h>
#include <U8g2lib.h>
#define I2C_ADDRESS 0x48

/* There are several ways to create your ADS1115_WE object:
 * ADS1115_WE adc = ADS1115_WE(); -> uses Wire / I2C Address = 0x48
 * ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS); -> uses Wire / I2C_ADDRESS
 * ADS1115_WE adc = ADS1115_WE(&Wire); -> you can pass any TwoWire object / I2C Address = 0x48
 * ADS1115_WE adc = ADS1115_WE(&Wire, I2C_ADDRESS); -> all together
 */
ADS1115_WE adc = ADS1115_WE(I2C_ADDRESS);
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 12, /* data=*/ 14, /* reset=*/ U8X8_PIN_NONE);

int voltageDivider_R1 = 51; 
int voltageDivider_R2 = 3.3;
float battery_max_voltage = 53;
float battery_min_voltage = 43;

void setup() 
{
  Wire.begin();
  u8g2.begin();
  Serial.begin(9600);

  if(!adc.init())
  {
    Serial.println("ADS1115 not connected!");
  }

  /* Set the voltage range of the ADC to adjust the gain
   * Please note that you must not apply more than VDD + 0.3V to the input pins!
   * 
   * ADS1115_RANGE_6144  ->  +/- 6144 mV
   * ADS1115_RANGE_4096  ->  +/- 4096 mV
   * ADS1115_RANGE_2048  ->  +/- 2048 mV (default)
   * ADS1115_RANGE_1024  ->  +/- 1024 mV
   * ADS1115_RANGE_0512  ->  +/- 512 mV
   * ADS1115_RANGE_0256  ->  +/- 256 mV
   */
  adc.setVoltageRange_mV(ADS1115_RANGE_4096); //comment line/change parameter to change range

  /* Set the inputs to be compared
   *  
   *  ADS1115_COMP_0_1    ->  compares 0 with 1 (default)
   *  ADS1115_COMP_0_3    ->  compares 0 with 3
   *  ADS1115_COMP_1_3    ->  compares 1 with 3
   *  ADS1115_COMP_2_3    ->  compares 2 with 3
   *  ADS1115_COMP_0_GND  ->  compares 0 with GND
   *  ADS1115_COMP_1_GND  ->  compares 1 with GND
   *  ADS1115_COMP_2_GND  ->  compares 2 with GND
   *  ADS1115_COMP_3_GND  ->  compares 3 with GND
   */
 adc.setCompareChannels(ADS1115_COMP_0_GND); //uncomment if you want to change the default

  /* Set number of conversions after which the alert pin will assert
   * - or you can disable the alert 
   *  
   *  ADS1115_ASSERT_AFTER_1  -> after 1 conversion
   *  ADS1115_ASSERT_AFTER_2  -> after 2 conversions
   *  ADS1115_ASSERT_AFTER_4  -> after 4 conversions
   *  ADS1115_DISABLE_ALERT   -> disable comparator / alert pin (default) 
   */
  //adc.setAlertPinMode(ADS1115_ASSERT_AFTER_1); //uncomment if you want to change the default

  /* Set the conversion rate in SPS (samples per second)
   * Options should be self-explaining: 
   * 
   *  ADS1115_8_SPS 
   *  ADS1115_16_SPS  
   *  ADS1115_32_SPS 
   *  ADS1115_64_SPS  
   *  ADS1115_128_SPS (default)
   *  ADS1115_250_SPS 
   *  ADS1115_475_SPS 
   *  ADS1115_860_SPS 
   */
  //adc.setConvRate(ADS1115_128_SPS); //uncomment if you want to change the default

  /* Set continuous or single shot mode:
   * 
   *  ADS1115_CONTINUOUS  ->  continuous mode
   *  ADS1115_SINGLE     ->  single shot mode (default)
   */
  //adc.setMeasureMode(ADS1115_CONTINUOUS); //uncomment if you want to change the default

   /* Choose maximum limit or maximum and minimum alert limit (window) in volts - alert pin will 
   *  assert when measured values are beyond the maximum limit or outside the window 
   *  Upper limit first: setAlertLimit_V(MODE, maximum, minimum)
   *  In max limit mode the minimum value is the limit where the alert pin assertion will be 
   *  be cleared (if not latched)  
   * 
   *  ADS1115_MAX_LIMIT
   *  ADS1115_WINDOW
   * 
   */
  adc.setAlertModeAndLimit_V(ADS1115_MAX_LIMIT, 3.6, 0.5); //uncomment if you want to change the default
  
  /* Enable or disable latch. If latch is enabled the alert pin will assert until the
   * conversion register is read (getResult functions). If disabled the alert pin assertion
   * will be cleared with next value within limits. 
   *  
   *  ADS1115_LATCH_DISABLED (default)
   *  ADS1115_LATCH_ENABLED
   */
  //adc.setAlertLatch(ADS1115_LATCH_ENABLED); //uncomment if you want to change the default

  /* Sets the alert pin polarity if active:
   *  
   * ADS1115_ACT_LOW  ->  active low (default)   
   * ADS1115_ACT_HIGH ->  active high
   */
  //adc.setAlertPol(ADS1115_ACT_LOW); //uncomment if you want to change the default
 
  /* With this function the alert pin will assert, when a conversion is ready.
   * In order to deactivate, use the setAlertLimit_V function  
   */
  //adc.setAlertPinToConversionReady(); //uncomment if you want to change the default

  Serial.println("ADS1115 Example Sketch - Single Shot Mode");
  Serial.println("Channel / Voltage [V]: ");
  Serial.println();
}

void loop() 
{
  float voltage = 0.0;
  float realVoltage = 0.0;
  
  

  Serial.print("A0: ");
  voltage = readChannel(ADS1115_COMP_0_GND);
  realVoltage = undivideVoltage( voltage, voltageDivider_R1, voltageDivider_R2);
  Serial.print(voltage);
  Serial.println();

  u8g2.clearBuffer();	
  u8g2.setFont(u8g2_font_7x14B_tr);	 // choose a suitable font
  u8g2.drawStr(0,10,"Input V:");
  u8g2.setCursor(60,10);
  u8g2.print( voltage, 2);


  u8g2.drawStr(0,30,"Real V:");
  u8g2.setCursor(60,30);
  u8g2.print( realVoltage, 2);

  float percentage = ( (realVoltage - battery_min_voltage) / (battery_max_voltage - battery_min_voltage) ) * 100;
  percentage  = constrain(percentage, 0.0, 100.0);
  
  u8g2.drawStr(60,50,"%");
  u8g2.setCursor(0,50);
  u8g2.print( percentage, 2);

  u8g2.drawFrame(0, 60, 128, 4);
  u8g2.drawBox(0, 60, (percentage / 100 ) * 128, 4);


  u8g2.sendBuffer();	  // transfer internal memory to the display

  delay(200);
}

float readChannel(ADS1115_MUX channel) {
  float voltage = 0.0;
  adc.setCompareChannels(channel);
  adc.startSingleMeasurement();
  while(adc.isBusy()){}
  voltage = adc.getResult_V(); // alternative: getResult_mV for Millivolt
  return voltage;
}

float undivideVoltage( float _dividedVoltage , int _r1 , int _r2 )
{
  return _dividedVoltage * 16.51877133105802;

  //return _dividedVoltage * ((_r1 + _r2) / _r2);
}



