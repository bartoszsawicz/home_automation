#include <RCSwitch.h>

#define RELAY1_ON_CODE 10005005
#define RELAY2_ON_CODE 10005006
#define RELAY3_ON_CODE 10005007
#define RELAY4_ON_CODE 10005000

#define RELAY1_OFF_CODE 10005020
#define RELAY2_OFF_CODE 10005022
#define RELAY3_OFF_CODE 10005023
#define RELAY4_OFF_CODE 10005016

#define RELAY1_PIN 3
#define RELAY2_PIN 4
#define RELAY3_PIN 5
#define RELAY4_PIN 6

RCSwitch mySwitch = RCSwitch();

void setup() {
  pinMode(RELAY1_PIN, OUTPUT);
  pinMode(RELAY2_PIN, OUTPUT);
  pinMode(RELAY3_PIN, OUTPUT);
  pinMode(RELAY4_PIN, OUTPUT);
  Serial.begin(9600);
  mySwitch.enableReceive(0);  // Receiver on interrupt 0 => that is pin #2

  digitalWrite(RELAY1_PIN, LOW);
  digitalWrite(RELAY2_PIN, LOW);
  digitalWrite(RELAY3_PIN, LOW);
  digitalWrite(RELAY4_PIN, LOW);
  Serial.println("Ready ");
}

void loop() {
  if (mySwitch.available()) {
    
    Serial.print("Received ");
    Serial.print( mySwitch.getReceivedValue() );
    Serial.print(" / ");
    Serial.print( mySwitch.getReceivedBitlength() );
    Serial.print("bit ");
    Serial.print("Protocol: ");
    Serial.println( mySwitch.getReceivedProtocol() );

    int receivedValue = mySwitch.getReceivedValue();
    switch(receivedValue) {
      case RELAY1_ON_CODE:
        digitalWrite(RELAY1_PIN, LOW);
      break;
      case RELAY2_ON_CODE:
        digitalWrite(RELAY2_PIN, LOW);
      break;
      case RELAY3_ON_CODE:
        digitalWrite(RELAY3_PIN, LOW);
      break;
      case RELAY4_ON_CODE:
        digitalWrite(RELAY4_PIN, LOW);
      break;
      case RELAY1_OFF_CODE:
        digitalWrite(RELAY1_PIN, HIGH);
      break;
      case RELAY2_OFF_CODE:
        digitalWrite(RELAY2_PIN, HIGH);
      break;
      case RELAY3_OFF_CODE:
        digitalWrite(RELAY3_PIN, HIGH);
      break;
      case RELAY4_OFF_CODE:
        digitalWrite(RELAY4_PIN, HIGH);
      break;      
    }
    mySwitch.resetAvailable();
  }
}
