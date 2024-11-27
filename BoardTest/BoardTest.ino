// the setup function runs once when you press reset or power the board
void setup() {
  // initialize pin 34 as an output
  pinMode(34, OUTPUT);
  // initialize pin 15 as an input
  pinMode(15, INPUT);
}

// the loop function runs over and over again forever
void loop() {
  // read the state of pin 15
  int pin15State = digitalRead(15);
  
  if (pin15State == HIGH) {  // if pin 15 is HIGH
    digitalWrite(34, HIGH);  // set pin 34 HIGH
  } else {
    digitalWrite(34, LOW);   // otherwise, set pin 34 LOW
  }
}
