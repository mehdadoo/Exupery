#include <Wire.h>

#if defined(ESP8266)
  // For ESP8266
  #define I2C_SDA D2
  #define I2C_SCL D1
#elif defined(ESP32)
  // For ESP32
  #define I2C_SDA 18
  #define I2C_SCL 16
#else
  #error "Unsupported board selected. Please select either ESP8266 or ESP32."
#endif

#define HMC5883L_ADDR 0x1E // I2C address of HMC5883

bool haveHMC5883L = false;

int16_t xOffset = -110;  // X-axis offset
int16_t yOffset = -316;  // Y-axis offset
int16_t zOffset = -509;  // Z-axis offset

int16_t xScale = 1122;   // X-axis scale
int16_t yScale = 720;    // Y-axis scale
int16_t zScale = 817;    // Z-axis scale

bool detectHMC5883L() {
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(10); // Select Identification register A
  Wire.endTransmission();
  Wire.requestFrom(HMC5883L_ADDR, 3);
  if (3 == Wire.available()) {
    char a = Wire.read();
    char b = Wire.read();
    char c = Wire.read();
    if (a == 'H' && b == '4' && c == '3') {
      return true;
    }
  }
  return false;
}

void setup() {
  Serial.begin(9600);
  Serial.println("HMC5883L TEST");
  
  Wire.begin(I2C_SDA, I2C_SCL);  // Initialize Wire library with specified pins
}

void loop() {
  bool detect = detectHMC5883L();

  if (!haveHMC5883L) {
    if (detect) {
      haveHMC5883L = true;
      Serial.println("We have HMC5883L, moving on");

      Wire.beginTransmission(HMC5883L_ADDR);
      Wire.write(0x02); // Select mode register
      Wire.write(0x00); // Continuous measurement mode
      Wire.endTransmission();
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
  
  Wire.beginTransmission(HMC5883L_ADDR);
  Wire.write(0x03); // Select register 3, X MSB register
  Wire.endTransmission();

  Wire.requestFrom(HMC5883L_ADDR, 6);
  if (6 <= Wire.available()) {
    x = Wire.read() << 8; // X MSB
    x |= Wire.read();      // X LSB
    z = Wire.read() << 8; // Z MSB
    z |= Wire.read();      // Z LSB
    y = Wire.read() << 8; // Y MSB
    y |= Wire.read();      // Y LSB
  }

  // Apply corrections
  x = (x - xOffset) * 1000 / xScale; // Apply offset and scale corrections
  y = (y - yOffset) * 1000 / yScale;
  z = (z - zOffset) * 1000 / zScale;

  //Serial.print("Corrected x: ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print(", ");
  Serial.println(z);

  delay(250);
}
