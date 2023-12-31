/*
  RoxEncoder is different from must encoder reading libraries because it's meant
  to be used with either the Pin on your arduino or a value read from a multiplexer

  Because of this you do not have to tell RoxEncoder what pins the encoder is connected
  to, instead you give RoxEncoder the state of your encoder, in other words, you
  read the pin state and RoxEncoder handles debouncing and determining the state.

  Because of this you must add the pinMode for your pin in your setup code

  Unlike the typical encoder library, RoxEncoder doesn't keep track of a "value"
  instead it just tells you if the encoder was rotated clockwise or counter-clockwise

  Then you are responsible for keeping track of what changes based on the direction
  of the rotation
*/
#include <RoxMux.h>

RoxEncoder encoder;

void setup(){
  // for this example pin 0 and 1 on my teensy are connected to an encoder
  pinMode(0, INPUT);
  pinMode(1, INPUT);
  // start the serial monitor and print a line
  Serial.begin(115200);
  Serial.println("RoxEncoder Example");
  // little delay before starting
  delay(100);
  // the .begin() method starts the debouncing timer
  encoder.begin();
  // tick sensitivity
  // this value should not exceed 3 or be less than 1
  // this value determines the number of ticks returned based on how fast
  // you rotate the encoder, the RoxEncoder.h for source code and how the values
  // will change.
  // a lower value is better for encoders without detents and depending on
  // how high a value you will be changing with the encoder
  encoder.setTickSensitivity(2);
}

void loop(){
  // 4 parameters passed to update method

  // #1 the actual state of the encoder Pin A, in this case we passs the output of digitalRead
  //    RoxEncoder doesn't read the pin itself so you have to pass the state of
  //    the pin, this is because RoxEncoder is meant to also work with Multiplexer
  //    outputs, so you have to read your pin or mux pin and then give that
  //    state to RoxEncoder and the class will do the job for you.
  //    ROXENCODER WILL DEBOUNCE THE READING, DO NO DEBOUNCE IT YOURSELF.

  // #2 same as #1 but for Pin B of the encoder

  // #3 (optional) the debounce time, this value is in milliseconds, default: 1ms
  //    this is how long to wait before reading the encoder again
  //    this is needed to clean some of the noise on the encoders.
  //    this value is 8-bits so keep it between 0 and 255

  // #4 (optional) the state of the encoder pins when they're active, default: LOW
  //    if your pin has a pullup resisitor then the active state will be LOW,
  //    if it's a pulldown resistor then the active state is HIGH.
  encoder.update(digitalRead(0), digitalRead(1), 2, LOW);

  // the update method will read the encoder and if can be used to test if
  // there was movement, however you can also use .read() after the update
  // to check if there was movement

  // .read() checks if the encoder was rotated
  if(encoder.read()){
    // encoder has been rotated
    Serial.print("Encoder rotated ");
    // .clockwise() tells us if the encoder was rotated clockwise
    // you can also use .clockwise()
    // if this method returns false the encoder was rotated counter-clockwise
    // encoder.getTicks() gives the number of calculated ticks, if you rotate
    // slowly it will return 1 the faster you rotate the higher the value.
    // encoder.getTicks() ranges from 1 to 9 depending on the sensitivity
    if(encoder.clockwise()){
      Serial.print("Clockwise ");
    } else {
      Serial.print("Counter-Clockwise ");
    }
    Serial.print(encoder.getTicks());
    Serial.print(" ticks");
  }
}
