#include "Arduino.h"
#include "DigiPotX9Cxxx.h"
#include "driver/gpio.h"

DigiPot::DigiPot() {}

void configurePin(uint8_t pin) 
{
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL << pin);
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&io_conf);
}

void DigiPot::setup(uint8_t incPin, uint8_t udPin, uint8_t csPin)
 {
  _incPin = incPin;
  _udPin = udPin;
  _csPin = csPin;  
  _currentValue = DIGIPOT_UNKNOWN;
  
	configurePin(_incPin);
	configurePin(_udPin);
	configurePin(_csPin);
	gpio_set_level((gpio_num_t)_csPin, 1); // Set CS high
}

DigiPot::DigiPot(uint8_t incPin, uint8_t udPin, uint8_t csPin) 
{
	_incPin = incPin;
	_udPin = udPin;
	_csPin = csPin;  
	_currentValue = DIGIPOT_UNKNOWN;

	configurePin(_incPin);
	configurePin(_udPin);
	configurePin(_csPin);
	gpio_set_level((gpio_num_t)_csPin, 1); // Set CS high

}

void DigiPot::reset() {
  // change down maximum number of times to ensure the value is 0
  decrease(DIGIPOT_MAX_AMOUNT);
  _currentValue = 0;
}

void DigiPot::set(uint8_t value)
 {
  value = constrain(value, 0, DIGIPOT_MAX_AMOUNT);
  if (_currentValue == DIGIPOT_UNKNOWN) reset();
  if (_currentValue > value)
{
    change(DIGIPOT_DOWN, _currentValue-value);
  } 
  else if (_currentValue < value)
  {
    change(DIGIPOT_UP, value-_currentValue);
  }
}

uint8_t DigiPot::get() 
{
  return _currentValue;
}

void DigiPot::increase(uint8_t amount) 
{
  amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
  change(DIGIPOT_UP, amount);
}

void DigiPot::decrease(uint8_t amount) 
{
  amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
  change(DIGIPOT_DOWN, amount);
}

void DigiPot::change(uint8_t direction, uint8_t amount)
 {
    amount = constrain(amount, 0, DIGIPOT_MAX_AMOUNT);
    gpio_set_level((gpio_num_t)_udPin, direction);  // Set direction
    gpio_set_level((gpio_num_t)_incPin, 1);         // Ensure INC is HIGH
    gpio_set_level((gpio_num_t)_csPin, 0);          // Select the chip

    for (uint8_t i = 0; i < amount; i++) 
	{
        gpio_set_level((gpio_num_t)_incPin, 0);     // Pulse INC LOW
        //ets_delay_us(1);               // Wait for minimum pulse duration
        gpio_set_level((gpio_num_t)_incPin, 1);     // Pulse INC HIGH
        //ets_delay_us(1);               // Wait for minimum pulse duration
        if (_currentValue != DIGIPOT_UNKNOWN) {
            _currentValue += (direction == DIGIPOT_UP ? 1 : -1);
            _currentValue = constrain(_currentValue, 0, DIGIPOT_MAX_AMOUNT);
        }
    }

    gpio_set_level((gpio_num_t)_csPin, 1);          // Deselect the chip
}

