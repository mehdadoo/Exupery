#include <Wire.h>

// I2C Pins
#define I2C_SDA 18
#define I2C_SCL 16

// Create a custom I2C bus
TwoWire custom_i2c_bus = TwoWire(0);

#define HMC5883L_ADDR 0x1E // I2C address of HMC5883

bool haveHMC5883L = false;

int16_t xOffset = -110;  // X-axis offset
int16_t yOffset = -316;  // Y-axis offset
int16_t zOffset = -509;  // Z-axis offset

int16_t xScale = 1122;   // X-axis scale
int16_t yScale = 720;    // Y-axis scale
int16_t zScale = 817;    // Z-axis scale

bool detectHMC5883L() 
{
  custom_i2c_bus.beginTransmission(HMC5883L_ADDR);
  custom_i2c_bus.write(10); // Select Identification register A
  custom_i2c_bus.endTransmission();
  custom_i2c_bus.requestFrom(HMC5883L_ADDR, 3);
  if (3 == custom_i2c_bus.available()) 
  {
    char a = custom_i2c_bus.read();
    char b = custom_i2c_bus.read();
    char c = custom_i2c_bus.read();
    if (a == 'H' && b == '4' && c == '3') 
    {
      return true;
    }
  }
  return false;
}

void setup() 
{
  Serial.begin(9600);
  Serial.println("GY271 TEST");
  
  // Initialize your custom I2C bus
  custom_i2c_bus.begin(I2C_SDA, I2C_SCL);

  // Uncomment the following line if pullup resistors are needed
  // custom_i2c_bus.setClockStretchLimit(2000);
}

void loop() {
  bool detect = detectHMC5883L();

  if (!haveHMC5883L) {
    if (detect) {
      haveHMC5883L = true;
      Serial.println("We have HMC5883L, moving on");

      custom_i2c_bus .beginTransmission(HMC5883L_ADDR);
      custom_i2c_bus .write(0x02); // Select mode register
      custom_i2c_bus .write(0x00); // Continuous measurement mode
      custom_i2c_bus .endTransmission();
    } else {
      Serial.println("No HMC5883L detected!");
      delay(2000);
      return;
    }
  } else {
    if (!detect) {
      haveHMC5883L = false;
      Serial.println("Lost connection to HMC5883L!");
      delay(2000);
      return;
    }
  }

  int16_t x, y, z; // Triple axis data
  
  custom_i2c_bus .beginTransmission(HMC5883L_ADDR);
  custom_i2c_bus .write(0x03); // Select register 3, X MSB register
  custom_i2c_bus .endTransmission();

  custom_i2c_bus .requestFrom(HMC5883L_ADDR, 6);
  if (6 <= custom_i2c_bus .available()) {
    x = custom_i2c_bus .read() << 8; // X MSB
    x |= custom_i2c_bus .read();      // X LSB
    z = custom_i2c_bus .read() << 8; // Z MSB
    z |= custom_i2c_bus .read();      // Z LSB
    y = custom_i2c_bus .read() << 8; // Y MSB
    y |= custom_i2c_bus .read();      // Y LSB
  }

  // Apply corrections
    x = (x - xOffset) * 1000 / xScale; // Apply offset and scale corrections
    y = (y - yOffset) * 1000 / yScale;
    z = (z - zOffset) * 1000 / zScale;

    Serial.print("Corrected x: ");
    Serial.print(x);
    Serial.print(" Corrected y: ");
    Serial.print(y);
    Serial.print(" Corrected z: ");
    Serial.println(z);

  delay(250);
}
