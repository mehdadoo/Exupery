#include <HardwareSerial.h>
HardwareSerial SerialPort(1); // use UART1

const int SERIAL_TX_PIN = 9;
const int SERIAL_RX_PIN = 11;


const int BUTTON_3_PIN = 3;
const int BUTTON_5_PIN = 5;
const int BUTTON_7_PIN = 7;
const int BUTTON_36_PIN = 36;
const int BUTTON_38_PIN = 38;
const int BUTTON_40_PIN = 40;

int prevButton3State = 2;
int prevButton5State = 2;
int prevButton7State = 2;
int prevButton36State = 2;
int prevButton38State = 2;
int prevButton40State = 2;

void setup() 
{
  Serial.begin(115200);
  pinMode(BUTTON_3_PIN, INPUT_PULLUP);
  pinMode(BUTTON_5_PIN, INPUT_PULLUP);
  pinMode(BUTTON_7_PIN, INPUT_PULLUP);
  pinMode(BUTTON_36_PIN, INPUT_PULLUP);
  pinMode(BUTTON_38_PIN, INPUT_PULLUP);
  pinMode(BUTTON_40_PIN, INPUT_PULLUP);

  SerialPort.begin(115200, SERIAL_8N1, SERIAL_TX_PIN, SERIAL_RX_PIN);


  // Print to Serial Monitor
    Serial.println("START|Connection|END");

    // Send the message over UART
    SerialPort.print("START|Connection|END");
}

void loop() 
{
  int button3State = digitalRead(BUTTON_3_PIN);
  int button5State = digitalRead(BUTTON_5_PIN);
  int button7State = digitalRead(BUTTON_7_PIN);
  int button36State = digitalRead(BUTTON_36_PIN);
  int button38State = digitalRead(BUTTON_38_PIN);
  int button40State = digitalRead(BUTTON_40_PIN);

  if (prevButton3State != button3State || prevButton5State != button5State || prevButton7State != button7State || prevButton36State != button36State || prevButton38State != button38State || prevButton40State != button40State) 
  {
    // Creating the message string
    String message = "START";
    message += "|B3:";
    message += button3State;
    message += "|B5:";
    message += button5State;
    message += "|B7:";
    message += button7State;
    message += "|B36:";
    message += button36State;
    message += "|B38:";
    message += button38State;
    message += "|B40:";
    message += button40State;
    message += "|END";

    // Print to Serial Monitor
    Serial.println(message);

    // Send the message over UART
    SerialPort.print(message);

    prevButton3State = button3State;
    prevButton5State = button5State;
    prevButton7State = button7State;
    prevButton36State = button36State;
    prevButton38State = button38State;
    prevButton40State = button40State;
  }
}
