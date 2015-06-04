/******************************************************
*OSURC Rover Pause Switch Firmware
*Written by Steven Gloyd (gloyds@onid.oregonstate.edu)
*Updated 4/22/2015
*******************************************************/


/*******************************************************************************
*                                  OPTIONS                                     *
********************************************************************************/

//Define macros for I/O
#define PAUSE_BUTTON 11	        //Pause Button
#define NPAUSE_BUTTON 9         //Unpause Button
#define STATUS_LED 12      	//Status LED
#define PWR_LED 8               //Power LED
#define PWR A0                  //Battery Voltage Input
#define XB 3			//XBee output

//Define macros for options
#define DEF_STATE 0 		//Default start state, 0 for pause/1 for run
#define BLINK_INTERVAL 500      //Delay between blinks of status led when paused
#define LOW_POWER_BLINK 100     //Delay between blinks of all LEDs when power is low

/*******************************************************************************
*				 CODE BODY                                     *
********************************************************************************/

//Set Variables to Options
int runningState = DEF_STATE;
int ledState = 1;
int batteryState;
long currMillis = 0;
long prevMillis = 0;

void setup() {
  //Serial.begin(9600);
  
  //Set inputs
  pinMode(PAUSE_BUTTON, INPUT);
  pinMode(NPAUSE_BUTTON, INPUT);
  
  //Set outputs
  pinMode(STATUS_LED, OUTPUT);
  pinMode(PWR_LED, OUTPUT);
  pinMode(XB, OUTPUT);
  
  //Write default state to XBee
  digitalWrite(XB, DEF_STATE);
  
  //Turn on Power LED
  digitalWrite(PWR_LED, HIGH);
  
  //Turn the Status LED on if the default state is pause
  if(DEF_STATE == 0)
    digitalWrite(STATUS_LED, HIGH);
}

void loop() {
  if(digitalRead(PWR) < 625){
     currMillis = millis();
     if(currMillis - prevMillis > LOW_POWER_BLINK){
       prevMillis = currMillis;
      
       if(ledState == LOW)
         ledState = HIGH;
       else if(ledState == HIGH)
         ledState = LOW;
        
       digitalWrite(STATUS_LED, ledState);
       digitalWrite(PWR_LED, ledState);
     }
  }
  else{
    //Read button and swap state
    if ((digitalRead(PAUSE_BUTTON) == HIGH) && (runningState == 1)) {
      runningState = 0;
      digitalWrite(XB, LOW);
      digitalWrite(STATUS_LED,  HIGH);
      delay(250);
    }
    else if ((digitalRead(NPAUSE_BUTTON) == HIGH) && (runningState == 0)) {
      runningState = 1;
      digitalWrite(XB, HIGH);
      delay(250);
    }
    
    //Change state of LED every BLINK_INTERVAL
    currMillis = millis();
    if ((currMillis - prevMillis > BLINK_INTERVAL) && (runningState == 1)) {
      prevMillis = currMillis;
      
      if(ledState == LOW)
        ledState = HIGH;
      else
        ledState = LOW;
        
      digitalWrite(STATUS_LED, ledState);
    }
  }
}
