/*
Arduino RC Drive code for the basic Rover chassis
Author: Nick McComb
Date: January 2015
*/

#include <SoftwareSerial.h>
#include <Sabertooth.h>

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
#define CH3_MAGNATUDE ((CH3_MAX - CH3_MIN) / 2)

#define CH5_MIN 918
#define CH5_MAX 1648
#define CH5_RANGE (CH5_MAX - CH5_MIN)

#define CH6_MIN 950   //Higher than actual, use < statement
#define CH6_MAX 1630  //Lower than actual, use > statement

#define STOP_CONSTANT 50  //Originally 20

SoftwareSerial SWSerial(NOT_A_PIN, 12); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth RearST(128, SWSerial); // Address 128, and use SWSerial as the serial port.
Sabertooth MidST(129, SWSerial);
Sabertooth FrontST(130, SWSerial);

void setup()
{
  pinMode(CHANNEL2, INPUT);
  pinMode(CHANNEL3, INPUT);
  pinMode(CHANNEL5, INPUT);
  pinMode(CHANNEL6, INPUT);
  
  Serial.begin(9600);
  Serial.println(CH2_STOP);
  
  
  SWSerial.begin(9600);
  RearST.autobaud();
  MidST.autobaud();
  FrontST.autobaud();
}

void loop()
{
  short lChannel = pulseIn(CHANNEL2, HIGH);
  lChannel -= CH2_STOP;
  
  short rChannel = pulseIn(CHANNEL3, HIGH);
  rChannel -= CH3_STOP;
  
  short enChannel = pulseIn(CHANNEL6, HIGH);
  
  short speedChannel = pulseIn(CHANNEL5, HIGH);
  //TO BE IMPLEMENTED LATER
  
  float lRatio;
  float rRatio;
  
  if(enChannel > CH6_MAX){  //Motors are good to go to be operated in a normal fashion
	  //Serial.println("MOTOR GO!");
	  
	  char lCommand = 0;
	  char lData = 0;
	  
	  char rCommand = 4;
	  char rData = 0;
	  
	  Serial.print("Left:  ");
	  if(abs(lChannel) > STOP_CONSTANT){  //Moving case
		lRatio = constrain(((float)lChannel / (float)CH2_MAGNATUDE), -1, 1);
		Serial.print(abs(lRatio) * 100);
		Serial.print("% ");
		if(lChannel > 0){
			Serial.print("forward  ");
			lCommand = 0;
			lData = constrain(((int)(lRatio * (float) 127)), 0, 127);
		}
		else {
			Serial.print("backward ");
			lRatio *= -1;  //Make the ratio positive
			lCommand = 1;
			lData = constrain(((int)(lRatio * (float) 127)), 0, 127);
		}
	  }
	  else{  //Not moving case
		  Serial.print("Stop            ");
		  lCommand = 1;
		  lData = 0;
	  }
	  
	  FrontST.command(lCommand, lData);
	  MidST.command(lCommand, lData);
	  RearST.command(lCommand, lData);
	  
	  Serial.print("Right: ");
	  if(abs(rChannel) > STOP_CONSTANT){  //Moving case
		rRatio = constrain(((float)rChannel / (float)CH3_MAGNATUDE), -1, 1);
		Serial.print(abs(rRatio) * 100);
		Serial.print("% ");
		if(rChannel > 0){  //Forward drive mode
			Serial.println("forward");
			rCommand = 4;
			rData = constrain((int)(rRatio * (float) 127), 0, 127);
		}
		else { //Backward Drive Mode
			Serial.println("backward");
			rRatio *= -1; //Make the ratio positive
			rCommand = 5;
			rData = constrain((int)(rRatio * (float) 127), 0, 127);
		}
	  }
	  else{  //Not moving case
		  Serial.println("Stop");
		  rCommand = 4;
		  rData = 0;
	  }

	  FrontST.command(rCommand, rData);
	  MidST.command(rCommand, rData);
	  RearST.command(rCommand, rData);


  }
  else {  //All motor stop
	  Serial.println("NO MOTOR :(");
  }
  
  delay(20);

  
}
