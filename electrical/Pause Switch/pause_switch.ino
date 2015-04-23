/******************************************************
*OSURC Rover Pause Switch Firmware
*Written by Steven Gloyd (gloyds@onid.oregonstate.edu)
*Updated 4/22/2015
*******************************************************/


/*******************************************************************************
*				     OPTIONS			       	       *		
********************************************************************************/

//Define macros for I/O
#define MOMENT_BUTTON 13 	//Momentary Button
#define BUTTON_LED 14 		//Button LED
#define POWER_LED 12		//Power LED
#define XB 					//XBee output

//Define macros for options
#define DEF_STATE 1 		//Default start state, 1 for pause/0 for run
#define DEF_BRIGHTNESS 255	//Default Brightness
#define MAX_BRIGHTNESS 255	//Maximum Brightness
#define MIN_BRIGHTNESS 0	//Minimum Brightness
#define FADE_RATE -5 		//Starting fade amount per loop
#define FADE_SPEED 30 		//Time between fades (ms)


/*******************************************************************************
*                                  CODE BODY                                   *		
********************************************************************************/

//Set variables to options
int pauseState = DEF_STATE;
int brightness = DEF_BRIGHTNESS;
int fade = FADE_RATE;

void setup(){
	//Set inputs
	pinMode(MOMENT_BUTTON, INPUT);
	//Set outputs
	pinMode(LED, OUTPUT);
	pinMode(XB, OUTPUT);
}

void loop(){
	//Read button and swap state
	if(digitalRead(MOMENT_BUTTON) && pauseState == 0){
		pauseState = 1;
		digitalWrite(XB, HIGH);
	}
	else if(digitalRead(MOMENT_BUTTON) && pauseState == 1){
		pauseState = 0;
		digitalWrite(XB, LOW);
		analogWrite(LED, MAX_BRIGHTNESS);
	}
	
	//If paused, fade button in and out.
	if(pauseState == 1){
		analogWrite(LED, brightness);
		
		brightness += fade;
		
		if(brightness == MIN_BRIGHTNESS || brightness == MAX_BRIGHTNESS){
			fade = -fade;
		}
		delay(FADE_SPEED);
	}
}
