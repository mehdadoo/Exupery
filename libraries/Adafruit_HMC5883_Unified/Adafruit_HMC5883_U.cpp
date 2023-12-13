

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif


#include <Wire.h>

#include <limits.h>

#include "Adafruit_HMC5883_U.h"

static float _hmc5883_Gauss_LSB_XY = 1100.0F; // Varies with gain
static float _hmc5883_Gauss_LSB_Z = 980.0F;   // Varies with gain


void Adafruit_HMC5883_Unified::write8(byte address, byte reg, byte value) {
  _wire->beginTransmission(address);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

byte Adafruit_HMC5883_Unified::read8(byte address, byte reg) {
  _wire->beginTransmission(address);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(address, static_cast<uint8_t>(1));
  return _wire->read();
}

void Adafruit_HMC5883_Unified::read() {
  _wire->beginTransmission(HMC5883_ADDRESS_MAG);
  _wire->write(HMC5883_REGISTER_MAG_OUT_X_H_M);
  _wire->endTransmission();
  _wire->requestFrom(HMC5883_ADDRESS_MAG, static_cast<uint8_t>(6));

  uint8_t xhi = _wire->read();
  uint8_t xlo = _wire->read();
  uint8_t zhi = _wire->read();
  uint8_t zlo = _wire->read();
  uint8_t yhi = _wire->read();
  uint8_t ylo = _wire->read();

  _magData.x = (float)(xlo | ((int16_t)xhi << 8));
  _magData.y = (float)(ylo | ((int16_t)yhi << 8));
  _magData.z = (float)(zlo | ((int16_t)zhi << 8));
}

/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/
Adafruit_HMC5883_Unified::Adafruit_HMC5883_Unified(int32_t sensorID) {
  _sensorID = sensorID;
  _wire = &Wire; // Use the default Wire instance
}

Adafruit_HMC5883_Unified::Adafruit_HMC5883_Unified(TwoWire *I2C, int32_t sensorID) {
  _sensorID = sensorID;
  _wire = I2C; // Use the provided I2C instance
}
/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/

/**************************************************************************/
/*!
    @brief  Setups the HW
*/
/**************************************************************************/
bool Adafruit_HMC5883_Unified::begin() {
  return begin(&Wire);
}

bool Adafruit_HMC5883_Unified::begin(TwoWire *I2C) {
  _wire = I2C;
  _wire->begin();

  // Set up the sensor here using the register definitions from the header file
  setMagGain(HMC5883_MAGGAIN_1_3); // Example gain value

  // Write to a register as an example of setup
  write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_MR_REG_M, 0x00);

  return true;
}

/**************************************************************************/
/*!
    @brief  Sets the magnetometer's gain
*/
/**************************************************************************/


void Adafruit_HMC5883_Unified::setMagGain(hmc5883MagGain gain) {
  write8(HMC5883_ADDRESS_MAG, HMC5883_REGISTER_MAG_CRB_REG_M, (byte)gain);

  _magGain = gain;

  switch (gain) {
  case HMC5883_MAGGAIN_1_3:
    _hmc5883_Gauss_LSB_XY = 1100;
    _hmc5883_Gauss_LSB_Z = 980;
    break;
  case HMC5883_MAGGAIN_1_9:
    _hmc5883_Gauss_LSB_XY = 855;
    _hmc5883_Gauss_LSB_Z = 760;
    break;
  case HMC5883_MAGGAIN_2_5:
    _hmc5883_Gauss_LSB_XY = 670;
    _hmc5883_Gauss_LSB_Z = 600;
    break;
  case HMC5883_MAGGAIN_4_0:
    _hmc5883_Gauss_LSB_XY = 450;
    _hmc5883_Gauss_LSB_Z = 400;
    break;
  case HMC5883_MAGGAIN_4_7:
    _hmc5883_Gauss_LSB_XY = 400;
    _hmc5883_Gauss_LSB_Z = 255;
    break;
  case HMC5883_MAGGAIN_5_6:
    _hmc5883_Gauss_LSB_XY = 330;
    _hmc5883_Gauss_LSB_Z = 295;
    break;
  case HMC5883_MAGGAIN_8_1:
    _hmc5883_Gauss_LSB_XY = 230;
    _hmc5883_Gauss_LSB_Z = 205;
    break;
  }
}

/**************************************************************************/
/*!
    @brief  Gets the most recent sensor event
*/
/**************************************************************************/
bool Adafruit_HMC5883_Unified::getEvent(sensors_event_t *event) {
  /* Clear the event */
  memset(event, 0, sizeof(sensors_event_t));

  /* Read new data */
  read();

  event->version = sizeof(sensors_event_t);
  event->sensor_id = _sensorID;
  event->type = SENSOR_TYPE_MAGNETIC_FIELD;
  event->timestamp = 0;
  event->magnetic.x =
      _magData.x / _hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.y =
      _magData.y / _hmc5883_Gauss_LSB_XY * SENSORS_GAUSS_TO_MICROTESLA;
  event->magnetic.z =
      _magData.z / _hmc5883_Gauss_LSB_Z * SENSORS_GAUSS_TO_MICROTESLA;

  return true;
}

/**************************************************************************/
/*!
    @brief  Gets the sensor_t data
*/
/**************************************************************************/
void Adafruit_HMC5883_Unified::getSensor(sensor_t *sensor) {
  /* Clear the sensor_t object */
  memset(sensor, 0, sizeof(sensor_t));

  /* Insert the sensor name in the fixed length char array */
  strncpy(sensor->name, "HMC5883", sizeof(sensor->name) - 1);
  sensor->name[sizeof(sensor->name) - 1] = 0;
  sensor->version = 1;
  sensor->sensor_id = _sensorID;
  sensor->type = SENSOR_TYPE_MAGNETIC_FIELD;
  sensor->min_delay = 0;
  sensor->max_value = 800;  // 8 gauss == 800 microTesla
  sensor->min_value = -800; // -8 gauss == -800 microTesla
  sensor->resolution = 0.2; // 2 milligauss == 0.2 microTesla
}
