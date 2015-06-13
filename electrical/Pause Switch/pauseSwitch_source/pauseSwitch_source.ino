/******************************************************
*OSURC Rover Pause Switch Firmware
*Written by Steven Gloyd (gloyds@onid.oregonstate.edu)
*Updated 6/4/2015
*******************************************************/


/*******************************************************************************
*                                  OPTIONS                                     *
********************************************************************************/

//Define macros for I/O
#define PAUSE_BUTTON 5	        //Pause Button
#define NPAUSE_BUTTON 6         //Unpause Button
#define STATUS_LED 3      	//Status LED
#define PWR_LED 2               //Power LED
#define PWR A0                  //Battery Voltage Input
#define XB 4			//XBee output

//Define macros for options
#define DEF_STATE 0 		//Default start state, 0 for pause/1 for run
#define BLINK_INTERVAL 500      //Delay between blinks of status led when paused
#define LOW_POWER_BLINK 150     //Delay between blinks of all LEDs when power is low

/*******************************************************************************
*				 CODE BODY                                     *
********************************************************************************/

//Set Variables to Options
int runningState = DEF_STATE;
int ledState = 0;
int battFlag = 0;
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
  if (DEF_STATE == 0)
    digitalWrite(STATUS_LED, HIGH);
}

void loop() {
  delay(1);
  //Serial.println(analogRead(PWR));
  if (analogRead(PWR) < 600) {
    battFlag = 1;
  }
  else if (analogRead(PWR) > 650) {
    battFlag = 0;
  }

  if (battFlag == 1) {
    currMillis = millis();
    if (currMillis - prevMillis > LOW_POWER_BLINK) {
      prevMillis = currMillis;

      if (ledState == LOW)
        ledState = HIGH;
      else if (ledState == HIGH)
        ledState = LOW;

      digitalWrite(STATUS_LED, ledState);
      digitalWrite(PWR_LED, ledState);
    }
  }
  else if (battFlag == 0) {
    digitalWrite(PWR_LED, HIGH);
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
      digitalWrite(STATUS_LED, LOW);
      delay(250);
    }

    //Change state of LED every BLINK_INTERVAL
    currMillis = millis();
    if (currMillis - prevMillis > BLINK_INTERVAL) {
      prevMillis = currMillis;
      if (runningState == 1) {
        if (ledState == LOW)
          ledState = HIGH;
        else
          ledState = LOW;

        digitalWrite(STATUS_LED, ledState);
      }
      else if (runningState == 0) {
        digitalWrite(STATUS_LED, HIGH);
      }
    }
  }
}
