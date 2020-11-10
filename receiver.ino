

// receiver.pde
//
// Simple example of how to use VirtualWire to receive messages
// Implements a simplex (one-way) receiver with an Rx-B1 module
//
// See VirtualWire.h for detailed API docs
// Author: Mike McCauley (mikem@airspayce.com)
// Copyright (C) 2008 Mike McCauley
// $Id: receiver.pde,v 1.3 2009/03/30 00:07:24 mikem Exp $

#include <VirtualWire.h>
#include <ServoTimer2.h>

ServoTimer2 leftWing;
ServoTimer2 rightWing;
ServoTimer2 mainmotor;
ServoTimer2 tailHoriz;
ServoTimer2 auxServo;

#define MAXAUXSER 3000
#define MINAUXSER 700

int auxServoValue = MINAUXSER;



int mainMotorIdle = 700;
int mainMotorCommand = 0;
int commandLW = 1500;
int commandRW = 1500;
int commandHT = 1500;
int commandVT = 1500;

const int led_pin = 13;
const int transmit_pin = 12;
const int receive_pin = 11;

int verif = 0;
int jgx = 0;
int jgy  = 0;
int jdx  = 0;
int jdy  = 0;
int trigg  = 0;
int trigd  = 0;
int btnA = 0;
int btnB = 0;
int btnX = 0;
int btnY = 0;
long lastValidCommand = 0;

void setup()
{
    delay(1000);
    //Serial.begin(9600);	// Debugging only
    //Serial.println("setup");

    // Initialise the IO and ISR
    vw_set_tx_pin(transmit_pin);
    vw_set_rx_pin(receive_pin);
    
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000);	 // Bits per sec

    vw_rx_start();       // Start the receiver PLL running

    pinMode(led_pin, OUTPUT);

    leftWing.attach(3);
    rightWing.attach(5);
    tailHoriz.attach(6);
    auxServo.attach(9);
    mainmotor.attach(10);

    auxServo.write(MINAUXSER);
}

void loop()
{
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;

  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    if (buflen == 11){
       updateCommands(buf);
    }
  }   

  ApplyButtonCommands();
  ApplyMainMotorCommands();
  ApplyAileronsCommands();
  
  if ((millis()-lastValidCommand)>2000)
  {
    //Serial.println("lostComm");
    mainMotorIdle = 700;
    mainmotor.write(700);
  }
  
}

void updateCommands(uint8_t buf[11])
{
  digitalWrite(led_pin, HIGH); // Flash a light to show received good message
  
  verif = (int) buf[0];
  if (verif == 126)
  {
    jgx = (int) buf[1];
    jgy = (int) buf[2];
    jdx = (int) buf[3];
    jdy = (int) buf[4];
    trigg = (int) buf[5];
    trigd = (int) buf[6];
  
    btnX = (int) buf[7]; 
    btnY = (int) buf[8]; 
    btnA = (int) buf[9]; 
    btnB = (int) buf[10]; 

    jgx = (jgx -33);
    jgy = (jgy -33);
    jdx = (jdx -33);
    jdy = (jdy -33);
    trigg = (trigg -33);
    trigd = (trigd -33);
    
    btnX = btnX - 48;
    btnY = btnY - 48;
    btnA = btnA - 48;
    btnB = btnB - 48;

    lastValidCommand = millis();
    digitalWrite(led_pin, LOW);
  }
}


void ApplyButtonCommands()
{
  if (btnA == 1){
    mainMotorIdle = 700;
  }

  if (btnB == 1){
    mainMotorIdle = 700;
  }

  if (btnY == 1){
    mainMotorIdle = 1100;
  }

  if (btnX == 1){
    if (auxServoValue == MINAUXSER){
      auxServoValue = MAXAUXSER;
    }  
    else{
      auxServoValue = MINAUXSER;
    }
    auxServo.write(auxServoValue);
  }
}


void ApplyMainMotorCommands()
{
  if (mainMotorIdle > 800){
    mainMotorCommand = trigd*10+mainMotorIdle;
    mainmotor.write(mainMotorCommand);
  }
  else{
    mainmotor.write(mainMotorIdle);
  }
}


void ApplyAileronsCommands()
{
  commandHT = (jdy*10)+1000;
  tailHoriz.write(commandHT);

  int delta = 0;
  int mid = (jdx*10)+1000;
  if (mid > 1451)
  {
    delta = 1900 - mid;
  }
  else if (mid < 1449)
  {
    delta = mid - 1000;
  }
  else
  {
    delta = 450;
  }

  if (jgy < 44)
  {
    commandLW = mid + delta*(44-jgy)/44;
    commandRW =  mid - delta*(44-jgy)/44;
  }

  else if (jgy > 46)
  {
    commandLW = mid - delta*(jgy-46)/44;
    commandRW =  mid + delta*(jgy-46)/44;
  }
  
  else 
  {
    commandLW = mid;
    commandRW =  mid;
  }
  
  leftWing.write(commandLW);
  rightWing.write(commandRW);  
}

