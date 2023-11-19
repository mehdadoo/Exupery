#include <HardwareSerial.h>
HardwareSerial SerialPort(1); // use UART1

const int BUTTON_3_PIN = 3;
const int BUTTON_5_PIN = 5;
const int BUTTON_7_PIN = 7;

int prevButton3State = 2;
int prevButton5State = 2;
int prevButton7State = 2;

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_3_PIN, INPUT);
  pinMode(BUTTON_5_PIN, INPUT);
  pinMode(BUTTON_7_PIN, INPUT);

  SerialPort.begin(115200, SERIAL_8N1, 9, 11);
}

void loop() 
{
  int button3State = digitalRead(BUTTON_3_PIN);
  int button5State = digitalRead(BUTTON_5_PIN);
  int button7State = digitalRead(BUTTON_7_PIN);

  if (prevButton3State != button3State || prevButton5State != button5State || prevButton7State != button7State) {
    // Creating the message string
    String message = "START|B3:";
    message += button3State;
    message += "|B5:";
    message += button5State;
    message += "|B7:";
    message += button7State;
    message += "|END";

    // Print to Serial Monitor
    Serial.println(message);

    // Send the message over UART
    SerialPort.print(message);

    prevButton3State = button3State;
    prevButton5State = button5State;
    prevButton7State = button7State;
  }
}
