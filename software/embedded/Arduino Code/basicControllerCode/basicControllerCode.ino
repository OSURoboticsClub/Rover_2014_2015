
#define CHANNEL2 2  //Vertical Left Stick
#define CHANNEL3 3  //Vertical Right Stick
#define CHANNEL5 5  //Speed control
#define CHANNEL6 6  //Enable

#define CH2_MIN 927
#define CH2_MAX 1654
#define CH2_STOP ((CH2_MAX-CH2_MIN)/2 + CH2_MIN)
#define CH2_MAGNATUDE ((CH2_MAX - CH2_MIN) / 2)

#define CH3_MIN 919
#define CH3_MAX 1644
#define CH3_STOP ((CH3_MAX-CH3_MIN)/2 + CH3_MIN)
#define CH2_MAGNATUDE ((CH3_MAX - CH3_MIN) / 2)

#define STOP_CONSTANT 20


void setup() {
  pinMode(CHANNEL2, INPUT);
  pinMode(CHANNEL3, INPUT);
  pinMode(CHANNEL5, INPUT);
  pinMode(CHANNEL6, INPUT);
  
  Serial.begin(9600);
  Serial.println(CH2_STOP);
}

void loop() {
  
  int lChannel = pulseIn(CHANNEL2, HIGH);
  lChannel -= CH2_STOP;
  
  float ratio;
  
  Serial.print("Left:  ");
  if(abs(lChannel) > STOP_CONSTANT){  //Moving case
	ratio = ((float)lChannel / (float)CH2_MAGNATUDE);
	Serial.print(abs(ratio) * 100);
	Serial.print("% ");
	if(lChannel > 0){
		Serial.println("forward");
	}
	else {
		Serial.println("backward");
	}
  }
  else{  //Not moving case
	  Serial.println("Stop");
  }
  
  
  
  delay(100);
  //if(lChannel )
  //Serial.println(pulseIn(CHANNEL2, HIGH) - CH2_STOP);

}
