#ifndef _Waterproof_Ultrasonic_h_
#define _Waterproof_Ultrasonic_h_

#include "arduino.h"

#define db_sa(...)        { Serial.println(__VA_ARGS__); }
#define db_sa_t(...)      {Serial.print(__VA_ARGS__);Serial.print("\t");}
#define db_sa(...)
#define db_sa_t(...)

#include "SoftwareSerial.h"

class Waterproof_Ultrasonic{
  public:
    Waterproof_Ultrasonic(uint8_t _Trig_RX_, uint8_t _Echo_TX_, uint8_t _mode_ = 1);
    float ping_cm();
    void begin();
    float readMode_1();
	  String getMode(){return String(mode);}
	  String getR27(){
		if(mode == 1) return "Empty";
		else if(mode == 2) return "47K";
		else if(mode == 3) return "120K";
	}
		
  private:
    unsigned long lastTime;
    uint8_t trigRX;
    uint8_t echoTX;
    uint8_t mode;
    int value_mm;
    float lastvalue_mm;
    
    const uint8_t numReadings = 4;
    int readings[4];      // the readings from the Serial
    int readIndex = 0;              // the index of the current reading
    int total = 0;                  // the running total
    
    SoftwareSerial * sieuam = 0;
    
    bool readSieuAm_available3(){
      bool temStatus = false;
      if(!sieuam)this->begin();
      while(sieuam->available()){
        readings[3] = readings[2];
        readings[2] = readings[1];
        readings[1] = readings[0];
        // read from the serial:
        readings[0] = sieuam->read();
        
        db_sa_t(readings[3], HEX);
        db_sa_t(readings[2], HEX);
        db_sa_t(readings[1], HEX);
        db_sa_t(readings[0], HEX);
        
        if(readings[3] == 0xff){
          int sum = readings[3] + readings[1] + readings[2];
          if((uint8_t)sum == readings[0]){
            value_mm = (readings[2] << 8) + readings[1];
            lastvalue_mm = value_mm;
            
            temStatus = true;
          }
        }
       }
        
      return temStatus;
    }
};

#endif
