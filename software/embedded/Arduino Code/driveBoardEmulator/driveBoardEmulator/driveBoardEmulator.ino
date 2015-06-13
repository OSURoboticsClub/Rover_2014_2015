const byte saberS1 = 10;
const byte ledPin = 13;

char id[7] = "DRIVE\n";
#include <SoftwareSerial.h>
#include <Sabertooth.h>

SoftwareSerial SWSerial(NOT_A_PIN, saberS1);
Sabertooth SHIT(128, SWSerial);


void setup() {
  //Initialize the sabertooth
  SWSerial.begin(9600);
  SHIT.autobaud();
  
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW); //Init as "not connected" state
  
  Serial.begin(9600);  //Configure HW Serial
  
  bool readyToGo = false;
  short readByte;
  while (!readyToGo) {
    if(Serial.available()){
      readByte = Serial.read();
      
      if(readByte == 'p')
        Serial.print(id);
      if(readByte == 'r'){
        readyToGo = true;
        digitalWrite(ledPin, HIGH);  //Signify "connected" state
      }
    }
  }
  
}

enum mainStates {CHECK_SERIAL, COMMAND_MOTORS};
enum packetParser {DRIVE_HEADER, LEFT_SPEED, RIGHT_SPEED, GIMBAL_PITCH, GIMBAL_YAW, GIMBAL_ROLL, DRIVE_CHECKSUM, DRIVE_FOOTER};
byte bufferIndex = 0;
char driveBuffer[8];
const byte DRIVE_PACKET_SIZE = 8;

struct driveData {
  driveData();
  byte leftSpeed;
  byte rightSpeed;
  bool freshData;
} parsedDriveData;


driveData::driveData(){
  freshData = false;
}

void loop() {
  static mainStates currentState = CHECK_SERIAL;
  switch(currentState){
    case CHECK_SERIAL:
      if(Serial.available()){
        driveBuffer[bufferIndex++] = Serial.read();
        
        if(bufferIndex == DRIVE_PACKET_SIZE){  //Then a packet needs to be parsed
          //This also accounts for the +1 error on the BB main code
          if(driveBuffer[DRIVE_HEADER] == 0xFF && driveBuffer[DRIVE_FOOTER] == 0xFF)  //Then we have a valid packet
            //PARSE THAT SHIT, SON
            digitalWrite(ledPin, LOW);
            
            parsedDriveData.freshData = true;
            
            parsedDriveData.leftSpeed = driveBuffer[LEFT_SPEED];
            parsedDriveData.rightSpeed = driveBuffer[RIGHT_SPEED];
            
            currentState = COMMAND_MOTORS;  //STATE CHANGE
        }
      }
      break;
    case COMMAND_MOTORS:
      if(parsedDriveData.freshData) {  //Check to see if there is new data to parse
        SHIT.motor(1, parsedDriveData.leftSpeed / 2);
        SHIT.motor(2, parsedDriveData.rightSpeed / 2);
      }
      currentState = CHECK_SERIAL;          //STATE CHANGE
      break;
  }
}











