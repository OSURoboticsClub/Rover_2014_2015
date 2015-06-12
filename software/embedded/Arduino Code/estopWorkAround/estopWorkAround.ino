char id [7] = "ESTOP\n";

void setup() {
  // put your setup code here, to run once:
  pinMode(8, INPUT);

  pinMode(13, OUTPUT);
  
  
  Serial.begin(9600);
  
  bool readyToGo = false;
  
  short readByte;
  
  
  while(!readyToGo){
  
    if(Serial.available()){
      readByte = Serial.read();
      
      if(readByte == 'p')
        Serial.print(id);
      if(readByte == 'r')
        readyToGo = true;
    }else {
      writeOutput();
    }
  }
}


void loop() {
  // put your main code here, to run repeatedly:
  writeOutput();
  
  delay(2);
}

void writeOutput(void){
  if(digitalRead(8)){
    digitalWrite(13, HIGH);
    Serial.print('1');
  }
  else {
    digitalWrite(13, LOW);
    Serial.print('0');
  }
}