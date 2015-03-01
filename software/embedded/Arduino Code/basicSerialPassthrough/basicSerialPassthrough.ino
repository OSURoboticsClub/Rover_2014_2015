#include <SoftwareSerial.h>

SoftwareSerial SaberComm(11, 10); //Rx, Tx

void setup() {
  // put your setup code here, to run once:
  SaberComm.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if( SaberComm.available() ) {
    Serial.print( SaberComm.read() );
  }

}
