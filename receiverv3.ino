
#define debug false
#define showCommands false
#define showOnlyGyro false


#include <VirtualWire.h>
#include <ServoTimer2.h>

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif



MPU6050 mpu;
#define OUTPUT_READABLE_YAWPITCHROLL

bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector





ServoTimer2 leftWing;
ServoTimer2 rightWing;
ServoTimer2 mainmotor;
ServoTimer2 tailHoriz;
ServoTimer2 tailVert;

#define MAXAUXSER 3000
#define MINAUXSER 700

int tailVertValue = MINAUXSER;

int mainMotorIdle = 700;
int mainMotorCommand = 0;
int commandLW = 0;
int commandRW = 0;
int commandHT = 0;
int commandVT = 0;

const int LED_PIN = 13;
const int transmit_pin = 12;
const int receive_pin = 11;

#define RED_LED_PIN 8
#define GREEN_LED_PIN 7
#define battWatcher A7

const int jgy_mpt = 47; // control middle points received
const int jgx_mpt = 44;
const int jdx_mpt = 47;
const int jdy_mpt = 43;

int verif = 0;
int jgx = jgx_mpt;
int jgy  = jgy_mpt;
int jdx  = jdx_mpt;
int jdy  = jdy_mpt;
int trigg  = 0;
int trigd  = 0;
int btnA = 0;
int btnB = 0;
int btnX = 0;
int btnY = 0;
long lastValidCommand = 0;

boolean autoStabiliser = false;

byte greenLedState = 1; // 0 : off, 1: on, 2: cycle 1s, 3: 3s, 4 : 4s
byte redLedState = 0; // 0 : off, 1: on, 2: cycle 1s, 3: 3s, 4 : 5s

// green 1 : MANUAL
// green 2 : AUTOPILOT ON
// green 3 : ?
// green 4 : NO SIGNAL

// red 1 : VERY LOW BATTERY
// red 2 : LOW BATTERY
// red 3 : MPU NO CONNECT
// red 4 : 

long lastGreenOn = millis();
long lastRedOn = millis();
long lastLedUpDate = millis();
long lastBattCheck = millis();

byte btnASum = 0;

// Asser
long lastPIUpdate = millis();

float yaw_offset = 0;
float pitch_offset = 0;
float roll_offset = 0;

int p_yaw = 2; // 5
int p_pitch = 2; // 5
int p_roll = 5; // 5

float i_yaw = 0; // 0.5
float i_pitch = 1; // 0.5
float i_roll = 0; // 0.5

int sum_yaw = 0;
int sum_roll = 0;
int sum_pitch = 0;



// ============================================================================================
// Set-up =====================================================================================
// ============================================================================================

void setup()
{
  delay(1000);

  // Initialise the IO and ISR
  vw_set_tx_pin(transmit_pin);
  vw_set_rx_pin(receive_pin);
  
  vw_set_ptt_inverted(true); // Required for DR3100
  vw_setup(2000);	 // Bits per sec

  vw_rx_start();       // Start the receiver PLL running
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(battWatcher, INPUT);
  digitalWrite(GREEN_LED_PIN, HIGH);
  digitalWrite(RED_LED_PIN, LOW);
  leftWing.attach(3);
  rightWing.attach(5);
  tailHoriz.attach(6);
  tailVert.attach(9);
  mainmotor.attach(10);

  tailVert.write(MINAUXSER);

  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  if ((debug)||(showCommands)){
    Serial.begin(115200);
  }
  // initialize device
  if (debug){
    Serial.println(F("Initializing I2C devices..."));
  }
  digitalWrite(GREEN_LED_PIN, LOW);
  mpu.initialize();

  // verify connection
  if (debug){
    Serial.println(F("Testing device connections..."));
    digitalWrite(GREEN_LED_PIN, HIGH);
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  }

  // load and configure the DMP
  if (debug){
    Serial.println(F("Initializing DMP..."));
  }
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
      // turn on the DMP, now that it's ready
      if (debug){
        Serial.println(F("Enabling DMP..."));
      }
      digitalWrite(GREEN_LED_PIN, LOW);
      mpu.setDMPEnabled(true);

      // enable Arduino interrupt detection
      if (debug){
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
      }
      digitalWrite(GREEN_LED_PIN, HIGH);
      attachInterrupt(0, dmpDataReady, RISING);
      mpuIntStatus = mpu.getIntStatus();

      // set our DMP Ready flag so the main loop() function knows it's okay to use it
      if (debug){
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
      }
      dmpReady = true;

      // get expected DMP packet size for later comparison
      packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
      digitalWrite(RED_LED_PIN, HIGH);
      redLedState = 1;
      // ERROR!
      // 1 = initial memory load failed
      // 2 = DMP configuration updates failed
      // (if it's going to break, usually the code will be 1)
      if (debug){
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
      }
  }
}



//==========================================================================================================================================================================
// MAIN LOOP ==========================================================================================================================================================
//==========================================================================================================================================================================

void loop()
{
  long temp = millis();
  
  // get Gyro data
  getGyroData();  
  
  uint8_t buf[VW_MAX_MESSAGE_LEN];
  uint8_t buflen = VW_MAX_MESSAGE_LEN;
  
  // Get data from controller
  if (vw_get_message(buf, &buflen)) // Non-blocking
  {
    if (buflen == 11){
       updateCommands(buf);
    }
  }   
  
  convertCommands();
  
  // Control motors with data
  ApplyButtonCommands();
  ApplyMainMotorCommands();
  
  ApplyAileronsCommands(commandRW, commandLW, commandHT,commandVT);



  // If controller is disconnected
  if (((temp-lastValidCommand)>1000))
  {
    //Serial.println("lostComm");
    mainMotorIdle = 700;
    mainmotor.write(700);
    jgx = jgx_mpt;
    jgy  = jgy_mpt;
    jdx  = jdx_mpt;
    jdy  = jdy_mpt;
    trigg = 0;
    trigd = 0;
    
    greenLedState = 4;
  }
  // If controller connected
  else {
    if (autoStabiliser){
      greenLedState = 2;
    }
    else {
      greenLedState = 1;
    }
  }


  // check battery
  if((temp-lastBattCheck)>1000){
    lastBattCheck = temp;
    batteryCheck();
  }

  // update leds
  if ((temp-lastLedUpDate)>250)
  {
    ledUpdate();
    lastLedUpDate = temp;
  } 
}

// greenLedState
// redLedState

// green 1 : MANUAL
// green 2 : AUTOPILOT ON
// green 3 : ?
// green 4 : NO SIGNAL

// red 1 : VERY LOW BATTERY
// red 2 : LOW BATTERY
// red 3 : MPU NO CONNECT (5s)
// red 4 : MPU FIFO overflow (3s)



//================================================================================
// Plane controls ================================================================
//================================================================================

void updateCommands(uint8_t buf[11])
{
  digitalWrite(LED_PIN, HIGH); // Flash a light to show received good message
  
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
    digitalWrite(LED_PIN, LOW);

    if (showCommands){
      Serial.print("JGX , JGY, JDX, JDY : ");
      Serial.print("\t");
      Serial.print(jgx);
      Serial.print("\t");
      Serial.print(jgy);
      Serial.print("\t");
      Serial.print(jdx);
      Serial.print("\t");
      Serial.print(jdy);
      Serial.print("\t");
      Serial.print("BTN : A B X Y : ");
      Serial.print("\t");
      Serial.print(btnA);
      Serial.print("\t");
      Serial.print(btnB);
      Serial.print("\t");
      Serial.print(btnX);
      Serial.print("\t");
      Serial.print(btnY);
      Serial.print("\t");
      Serial.print("trig d g : ");
      Serial.print("\t");
      Serial.print(trigd);
      Serial.print("\t");
      Serial.println(trigg);
      
    }
    
  }
}

void convertCommands(){
  
  if (!autoStabiliser){
    commandLW = jgx_mpt-jgx;
    commandRW = jgx-jgx_mpt;
  
    if (jdy > 70){
      commandLW = -45;
      commandRW = -45;
    }
    else if(jdy < 20){
      commandLW = 45;
      commandRW = 45;
    }
    
    commandHT = jgy_mpt-jgy;
    commandVT = jdx_mpt-jdx;
  
    
    if (trigg > 45){
      commandLW = commandLW / 2;
      commandRW = commandRW / 2;
      commandHT = commandHT / 2;
      commandVT = commandVT / 2;
      
    }
  }

  if (autoStabiliser){
    if ((millis() - lastPIUpdate) > 100){
      lastPIUpdate= millis();
      commandVT = jdx_mpt-jdx;
      int commandPitch = 0;
      int commandRoll = 0;
      //ypr[0] = ypr[0]*57-yaw_offset;
      ypr[1] = ypr[1]*57-pitch_offset;
      ypr[2] = ypr[2]*57-roll_offset;

      //int current_yaw = (int) ypr[0];
      int current_pitch = (int) ypr[1];
      int current_roll = (int) ypr[2];

      //int askedAngleYaw = jdx - jdx_mpt; ???

      int askedAnglePitch = -jgy + jgy_mpt;
      int askedAngleRoll = jgx - jgx_mpt;

      if (trigg > 19){
        int factor = trigg / 18;
        askedAnglePitch = askedAnglePitch/factor;
        askedAngleRoll = askedAngleRoll/factor;
      
      }
      
      int errorPitch = askedAnglePitch - current_pitch;
      int errorRoll = askedAngleRoll - current_roll;

      commandPitch = errorPitch*p_pitch + sum_pitch*i_pitch;
      commandRoll = errorRoll*p_roll + sum_roll*i_roll;

      if (commandPitch > 49){
        commandHT = 50;
      }
      else if (commandPitch < -49){
        commandHT = -50;
      }
      else{
        commandHT = commandPitch;
        sum_pitch = sum_pitch + errorPitch;
      }

      
      
      if (commandRoll > 49){
        commandLW = -50;
        commandRW = 50;
      }
      else if (commandRoll < -49){
        commandLW = 50;
        commandRW = -50;
      }
      else{
        commandLW = -commandRoll;
        commandRW = commandRoll;
        sum_roll = sum_roll + errorRoll;
      }
      

    }
  }
}

void ApplyButtonCommands()
{
  if (btnA == 1){
    sum_yaw = 0;
    sum_roll = 0;
    sum_pitch = 0;
    autoStabiliser = true;
    p_pitch = analogRead(A6)/100;
    i_pitch = analogRead(A7)/100;
    btnASum = btnASum + 1;
  }
  else{
    btnASum = 0;
  }
  if (btnB == 1){
    mainMotorIdle = 700;
  }

  if (btnY == 1){
    mainMotorIdle = 1100;
  }

  if (btnX == 1){
    sum_yaw = 0;
    sum_roll = 0;
    sum_pitch = 0;
    autoStabiliser = false;
  }
  
  if (btnASum > 15){
    btnASum = 0;
    yaw_offset = ypr[0]*57;
    pitch_offset = ypr[1]*57;
    roll_offset = ypr[2]*57;
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


void ApplyAileronsCommands(int RWS_Value, int LWS_Value, int HTS_Value, int VTS_Value)
{
  int commandTailHoriz;
  int commandTailVert;
  int commandLWing;
  int commandRWing;
  // 50 is aileron up
  // 0 is neutral
  // -50 is aileron down

  // for tail : 0 is middle
  // -50 is aileron left, 50 is aileron right
  const int servomax = 50;
  const int servomin = -50;
  const int servoMidPoint = 1450;
  const int multiplier = 10;
  
  if (HTS_Value > servomax)
  {
    HTS_Value = servomax;
  }
  else if (HTS_Value < servomin)
  {
    HTS_Value = servomin;
  }

  if (VTS_Value > servomax)
  {
    VTS_Value = servomax;
  }
  else if (VTS_Value < servomin)
  {
    VTS_Value = servomin;
  }

  if (RWS_Value > servomax)
  {
    RWS_Value = servomax;
  }
  else if (RWS_Value < servomin)
  {
    RWS_Value = servomin;
  }

  if (LWS_Value > servomax)
  {
    LWS_Value = servomax;
  }
  else if (LWS_Value < servomin)
  {
    LWS_Value = servomin;
  }

  // tail horiz
  commandTailHoriz = (HTS_Value*multiplier)+servoMidPoint; // Sens 1
  //commandTailHoriz = servoMidPoint-(HTS_Value*9); // Sens 2
  
  tailHoriz.write(commandTailHoriz);


  // tail vert
  commandTailVert = (VTS_Value*multiplier)+servoMidPoint; // Sens 1
  //commandTailVert = servoMidPoint-(VTS_Value*9); // Sens 2

  tailVert.write(commandTailVert);

  
  // wing left
  commandLWing = (LWS_Value*multiplier)+servoMidPoint; // Sens 1
  //commandLWing = servoMidPoint-(LWS_Value*9); // Sens 2

  leftWing.write(commandLWing);

  // wing right
  //commandRWing = (RWS_Value*multiplier)+servoMidPoint; // Sens 1
  commandRWing = servoMidPoint-(RWS_Value*9); // Sens 2

  rightWing.write(commandRWing);
}




//================================================================================
// LED Update    ================================================================
//================================================================================

void updateGreenLed ()
{
  long temp = millis();
  
  if (greenLedState == 0){
    digitalWrite(GREEN_LED_PIN, LOW);
  }
  
  else if (greenLedState == 1){
    digitalWrite(GREEN_LED_PIN, HIGH);
  }

  else if (greenLedState == 2){
    if ((temp - lastGreenOn)>1000){
      digitalWrite(GREEN_LED_PIN, HIGH);
      lastGreenOn = temp;
    }
    else if ((temp - lastGreenOn)>500){
      digitalWrite(GREEN_LED_PIN, LOW);
    }
  }

  else if (greenLedState == 3){
    if ((temp - lastGreenOn)>3000){
      digitalWrite(GREEN_LED_PIN, HIGH);
      lastGreenOn = temp;
    }
    else if ((temp - lastGreenOn)>1500){
      digitalWrite(GREEN_LED_PIN, LOW);
    }
  }
  
  else if (greenLedState == 4){
    if ((temp - lastGreenOn)>4000){
      digitalWrite(GREEN_LED_PIN, HIGH);
      lastGreenOn = temp;
    }
    else if ((temp - lastGreenOn)>2000){
      digitalWrite(GREEN_LED_PIN, LOW);
    }
  }
}


void updateRedLed ()
{
  long temp = millis();
  
  if (redLedState == 0){
    digitalWrite(RED_LED_PIN, LOW);
  }
  
  else if (redLedState == 1){
    digitalWrite(RED_LED_PIN, HIGH);
  }

  else if (redLedState == 2){
    if ((temp - lastRedOn)>1000){
      digitalWrite(RED_LED_PIN, HIGH);
      lastRedOn = temp;
    }
    else if ((temp - lastRedOn)>500){
      digitalWrite(RED_LED_PIN, LOW);
    }
  }

  else if (redLedState == 3){
    if ((temp - lastRedOn)>5000){
      digitalWrite(RED_LED_PIN, HIGH);
      lastRedOn = temp;
    }
    else if ((temp - lastRedOn)>2500){
      digitalWrite(RED_LED_PIN, LOW);
    }
  }

  else if (redLedState == 4){
    if ((temp - lastRedOn)>3000){
      digitalWrite(RED_LED_PIN, HIGH);
      lastRedOn = temp;
    }
    else if ((temp - lastRedOn)>1500){
      digitalWrite(RED_LED_PIN, LOW);
    }
  }
}


void ledUpdate(){
  if ((debug)&&(!showOnlyGyro)){
      Serial.println(" ");
      Serial.print("R Led : ");
      Serial.print(redLedState);
      Serial.print("G Led : ");
      Serial.println(greenLedState);
    }
    updateRedLed();
    updateGreenLed();
}

void batteryCheck(){
  int battLvl = analogRead(battWatcher);
  if ((debug)&&(!showOnlyGyro)){
    Serial.println(" ");
    Serial.print("Battery Level : ");
    Serial.println(battLvl);
  }

  if (battLvl < 660){
    redLedState = 1; //very low batt (9.6V)
  }
  else if (battLvl < 730){
    redLedState = 2; //low batt (10.6V)
  } 
  else if ((redLedState != 3)&&(redLedState != 4)){
    redLedState = 0;
  }
}

//================================================================================
// Gyro Update    ================================================================
//================================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



void getGyroData()
{
  if (!dmpReady){
    // if gyro disabled
    autoStabiliser = false;
    redLedState = 3;
    return;
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
      // reset so we can continue cleanly
      mpu.resetFIFO();
      redLedState = 4;
      if (debug){
        Serial.println(F("FIFO overflow!"));
      }
  } 
  else if (mpuIntStatus & 0x02) 
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
      
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
      
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    if (debug){
      Serial.print("ypr\t");
      Serial.print(ypr[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(ypr[1] * 180/M_PI);
      Serial.print("\t");
      Serial.println(ypr[2] * 180/M_PI);
    }

  }

}
