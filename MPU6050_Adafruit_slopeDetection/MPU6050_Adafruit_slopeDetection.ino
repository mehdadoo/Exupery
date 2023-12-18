// Basic demo for accelerometer readings from Adafruit MPU6050

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;

void setup(void) 
{
  Serial.begin(115200);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) 
    {
      delay(10);
    }
  }
  //Serial.println("MPU6050 Found!");

	mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
	mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);
  mpu.setGyroStandby(true, true, true);
  mpu.setAccelerometerStandby(true, false, true);

  mpu.enableCycle(true);
  mpu.setCycleRate( MPU6050_CYCLE_20_HZ );

  delay(100);
}

void loop() 
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  Serial.print(a.acceleration.y);
  Serial.print(", ");
  Serial.print(temp.temperature);
  Serial.println("");
  delay(50);
}