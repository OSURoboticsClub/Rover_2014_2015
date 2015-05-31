/*
Arduino RC Drive code for the basic Rover chassis

Takes input from an RC Reciever and outputs to three connected saberteeth, with different addresses. 
You can find this documented below.

Author: Nick McComb [mccombn@onid.oregonstate.edu]
Date: January 2015
*/

#include <SoftwareSerial.h>
#include <Sabertooth.h>

//Comment the following line out if you do not want serial debug statements
#define DEBUG

#define CHANNEL2 2  //Vertical Left Stick
#define CHANNEL3 3  //Vertical Right Stick
#define CHANNEL5 5  //Speed control
#define CHANNEL6 6  //Enable

#define CH2_MIN 1048
#define CH2_MAX 1868
#define CH2_STOP ((CH2_MAX-CH2_MIN)/2 + CH2_MIN)
#define CH2_MAGNATUDE ((CH2_MAX - CH2_MIN) / 2)

#define CH3_MIN 1075
#define CH3_MAX 1888
#define CH3_STOP ((CH3_MAX-CH3_MIN)/2 + CH3_MIN)
#define CH3_MAGNATUDE ((CH3_MAX - CH3_MIN) / 2)

#define CH5_MIN 918
#define CH5_MAX 1648
#define CH5_RANGE (CH5_MAX - CH5_MIN)

#define CH6_MIN 950   //Higher than actual, use < statement
#define CH6_MAX 1630  //Lower than actual, use > statement

//This defines how far away from the "CH*_MAGNATUDE" the motors will actually stop
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
  #ifdef DEBUG
    Serial.println("[Board Initializing...]");
  #endif
  
  
  SWSerial.begin(9600);
  RearST.autobaud();
  MidST.autobaud();
  FrontST.autobaud();
  
  #ifdef DEBUG
    Serial.println("[Board Initialization Complete]");
  #endif
}

void loop()
{
  short lChannel = pulseIn(CHANNEL2, HIGH);
  #ifdef DEBUG
    Serial.print(lChannel);
    Serial.print(" ");
  #endif
  lChannel -= CH2_STOP;
  
  short rChannel = pulseIn(CHANNEL3, HIGH);
  //Serial.print(rChannel);
  //Serial.print(" ");

  rChannel -= CH3_STOP;
  
  short enChannel = pulseIn(CHANNEL6, HIGH);
  
  short speedChannel = pulseIn(CHANNEL5, HIGH);
  
  //##! Begin SUPER DUPER HACKY SECTION !##
  //Swap sides
  short temp = rChannel;
  rChannel = lChannel;
  lChannel = temp;
  
  
  //Flip both directions
  rChannel *= -1;  
  lChannel *= -1;  
  //##! End SUPER DUPER HACKY SECTION !##
  
  float lRatio;
  float rRatio;
  
  if(enChannel > CH6_MAX){  //Motors are good to go to be operated in a normal fashion (e.g. ESTOP is not activated)
	  //Serial.println("MOTOR GO!");
	  
	  char lCommand = 0;
	  char lData = 0;
	  
	  char rCommand = 4;  
	  char rData = 0;
	   
    #ifdef DEBUG
	    Serial.print("Left:  ");
    #endif
	  if(abs(lChannel) > STOP_CONSTANT){  //Moving case
      lRatio = constrain(((float)lChannel / (float)CH2_MAGNATUDE), -1, 1);
      #ifdef DEBUG
        Serial.print(abs(lRatio) * 100);
        Serial.print("% ");
      #endif
      if(lChannel > 0){
        #ifdef DEBUG
          Serial.print("forward  ");
        #endif
        lCommand = 0;
        lData = constrain(((int)(lRatio * (float) 127)), 0, 127);
      }
      else {
        #ifdef DEBUG
          Serial.print("backward ");
        #endif
        lRatio *= -1;  //Make the ratio positive
        lCommand = 1;
        lData = constrain(((int)(lRatio * (float) 127)), 0, 127);
      }
	  }
	  else{  //Not moving case
      #ifdef DEBUG
		    Serial.print("Stop            ");
      #endif
		  lCommand = 1;
		  lData = 0;
	  }
	  
	  FrontST.command(lCommand, lData);
	  MidST.command(lCommand, lData);
	  RearST.command(lCommand, lData);
	  
    #ifdef DEBUG
	    Serial.print("Right: ");
    #endif
	  if(abs(rChannel) > STOP_CONSTANT){  //Moving case
      rRatio = constrain(((float)rChannel / (float)CH3_MAGNATUDE), -1, 1);
      #ifdef DEBUG
        Serial.print(abs(rRatio) * 100);
        Serial.print("% ");
      #endif
      if(rChannel > 0){  //Forward drive mode
        #ifdef DEBUG
          Serial.println("forward");
        #endif
        rCommand = 4;
        rData = constrain((int)(rRatio * (float) 127), 0, 127);
      }
      else { //Backward Drive Mode
        #ifdef DEBUG
          Serial.println("backward");
        #endif
        rRatio *= -1; //Make the ratio positive
        rCommand = 5;
        rData = constrain((int)(rRatio * (float) 127), 0, 127);
      }
	  }
    else{  //Not moving case
      #ifdef DEBUG
        Serial.println("Stop");
      #endif
      rCommand = 4;
      rData = 0;
    }

    FrontST.command(rCommand, rData);
    MidST.command(rCommand, rData);
    RearST.command(rCommand, rData);


  }
  else {  //All motor stop
    #ifdef DEBUG
	    Serial.println("NO MOTOR :(");
    #endif

    FrontST.command(4, 0);
    MidST.command(4, 0);
    RearST.command(4, 0);
          
    FrontST.command(0, 0);
    MidST.command(0, 0);
    RearST.command(0, 0);
  }
  
  delay(20);  
}
