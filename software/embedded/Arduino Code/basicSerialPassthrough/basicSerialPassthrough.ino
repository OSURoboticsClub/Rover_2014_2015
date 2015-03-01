#include <SoftwareSerial.h>

SoftwareSerial SaberComm(11, 10); //Rx, Tx

void setup() {
  // put your setup code here, to run once:
  SaberComm.begin(9600);
  Serial.begin(9600);
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);
}

void loop() {
  if( SaberComm.available() ) {
    Serial.print( (char) SaberComm.read() );
    digitalWrite(13, HIGH);
  }

}
