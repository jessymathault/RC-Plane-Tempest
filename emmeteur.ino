/*
 Controlling a servo position using a potentiometer (variable resistor)
 by Michal Rinott <http://people.interaction-ivrea.it/m.rinott>

 modified on 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Knob
*/
#include <VirtualWire.h>
#include <math.h>

const int transmit_pin = 11;

int potpin0 = 0;  // analog pin used to connect the potentiometer
int potpin1 = 1;
int potpin2 = 2;
int potpin3 = 3;
int potpin4 = 4;
int potpin5 = 5;

int val0;    // variable to read the value from the analog pin
int val1;
int val2;
int val3;
int val4;
int val5;

int JGx;
int JGy;
int JDx;
int JDy;
int trigD;
int trigG;

int BY;
int BX;
int BA;
int BB;


char jgx = 'c';
char jgy  = 'c';
char jdx  = 'c';
char jdy  = 'c';
char trigg  = 'c';
char trigd  = 'c';
char btnA = 'c';
char btnB = 'c';
char btnX = 'c';
char btnY = 'c';


int boutonA = 7;
int boutonB = 8;
int boutonX = 6;
int boutonY = 5;


void setup() {
  pinMode(potpin0, INPUT);
  pinMode(potpin1, INPUT);
  pinMode(potpin2, INPUT);
  pinMode(potpin3, INPUT);
  pinMode(potpin4, INPUT);
  pinMode(potpin5, INPUT);

  pinMode(boutonA, INPUT);
  pinMode(boutonB, INPUT);

  pinMode(boutonX, INPUT);
  pinMode(boutonY, INPUT);

  Serial.begin(9600);

  vw_set_tx_pin(transmit_pin);
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);
}

void loop() {
  //int maximum = round(sqrt(1023));
  int maximum = 1023;

  
  val0 = analogRead(potpin0);            // reads the value of the potentiometer (value between 0 and 1023)
  //val0 = round(sqrt(val0));
  val0 = map(val0, 0, maximum, 0, 90);     // scale it to use it with the servo (value between 0 and 180)

  val1 = analogRead(potpin1);            // reads the value of the potentiometer (value between 0 and 1023)
  //val1 = round(sqrt(val1));
  val1 = map(val1, 0, maximum, 90, 0);  
  
  val2 = analogRead(potpin2);            // reads the value of the potentiometer (value between 0 and 1023)
  //val2 = round(sqrt(val2));
  val2 = map(val2, 0, maximum, 0, 90);  
  
  val3 = analogRead(potpin3);            // reads the value of the potentiometer (value between 0 and 1023)
  val3 = map(val3, 0, 1023, 0, 90);  

  val4 = analogRead(potpin4);            // reads the value of the potentiometer (value between 0 and 1023)
  //val4 = round(sqrt(val4));
  val4 = map(val4, 0, maximum, 90, 0);  

  val5 = analogRead(potpin5);            // reads the value of the potentiometer (value between 0 and 1023)
  val5 = map(val5, 0, maximum, 0, 90);  

  BY = digitalRead(boutonY); 
  BB = digitalRead(boutonB); 
  BX = digitalRead(boutonX);
  BA = digitalRead(boutonA); 

  
  JGx = val0;
  JGy = val1;

  JDx = val4;
  JDy = val2;

  trigD = val3;
  trigG = val5;

//  Serial.print("JG(");
//  Serial.print(JGx);
//  Serial.print(", ");
//  Serial.print(JGy);
//  Serial.print(") JD(");
//  Serial.print(JDx);
//  Serial.print(", ");
//  Serial.print(JDy);
//  Serial.print(") TG(");
//  Serial.print(trigG);
//  Serial.print(") TD(");
//  Serial.print(trigD);
//  Serial.print(") ");

  jgx = (char) (JGx+33);
  jgy = (char) (JGy+33);
  jdx = (char) (JDx+33);
  jdy = (char) (JDy+33);
  trigg = (char) (trigG+33);
  trigd = (char) (trigD+33);

  btnX = '0'; 
  btnY = '0'; 
  btnA = '0'; 
  btnB = '0'; 
  
  if (BB == HIGH){
    //Serial.print("B ");
    btnB = '1'; 
  }

  if (BA == HIGH){
    //Serial.print("A ");
    btnA = '1'; 
  }
  
  if (BY == HIGH){
    //Serial.print("Y ");
    btnY = '1'; 
  }
  
  if (BX == HIGH){
    //Serial.print("X ");
    btnX = '1'; 
  }

//  Serial.println(" ");
  
    
  char msg[11] = {'~', jgx, jgy, jdx, jdy, trigg, trigd, btnX, btnY, btnA, btnB};
  
  vw_send((uint8_t *)msg, 11);
  vw_wait_tx(); // Wait until the whole message is gone
  
  Serial.println(msg);         
  delay(25);                           // waits for the servo to get there
}

