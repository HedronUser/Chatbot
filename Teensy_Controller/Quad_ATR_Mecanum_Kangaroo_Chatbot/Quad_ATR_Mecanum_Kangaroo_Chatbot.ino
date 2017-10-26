//=========================HEADER=============================================================
/*
   Quad ATR Mecanum with Kangaroo
   AUTHOR: Jason Traud
   DATE: 6/8/2015
   
   This firmware demonstrates basic movement routines on a Mecanum ATR with Kangaroo
   motion controller.
   
   Hardware: 
   
     Arduino Uno R3       (MCU-050-000)
     Programmable Mecanum (TP-095-004) 
     Sabertooth 2x25      (TE-091-225)
     Kangaroo             (TE-180-000)

   Connections:    
     
     Arduino D1   -   Sabertooth S1 (Serial Tx on Arduino to the Rx on the Kangaroo)
     Arduino D0   -   Sabertooth S2 (Serial Rx on Arduino to the Tx on the Kangaroo)
     Arduino Gnd  -   Sabertooth 0V (A common ground is needed for stable communication) 
             
			
   License: CCAv3.0 Attribution-ShareAlike (http://creativecommons.org/licenses/by-sa/3.0/)
   You're free to use this code for any venture. Attribution is greatly appreciated. 
//============================================================================================
Spektrum DX5e:
http://www.superdroidrobots.com/shop/item.aspx/spektrum-dx5etransmitter-with-ar610-receiver/992/

*/

// ****************************************************
// Libraries
// ****************************************************
#include <Kangaroo.h>
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3
#include <ArduinoJson.h>

// ****************************************************
// Motor Controller Initialization
// ****************************************************
//truths
// white wire is S1 front
// blue wire is S2 front

//white wire is S1 rear
//yellow wire is S2 rear

// Teensy TX (pin 11) goes to Kangaroo S1
// Teensy RX (pin 10) goes to Kangaroo S2
// Teensy GND         goes to Kangaroo 0V
// Teensy 5V input    goes to Kangaroo 5V (OPTIONAL, if you want Kangaroo to power the Teensy)
#define TX2_PIN 10  // blue wire
#define RX2_PIN 9 //white wire
#define TX3_PIN 8
#define RX3_PIN 7

// Assign the serial COM to the Kangaroo as Serial1 on the Teensy
SoftwareSerial  SerialPort1(RX2_PIN, TX2_PIN);
SoftwareSerial  SerialPort2(RX3_PIN, TX3_PIN);

KangarooSerial  K1(SerialPort1);
KangarooSerial  K2(SerialPort2);


// Initialize our Kangroo objects. The named channels (1, 2, 3, or 4) must be configured on
// the Kangroo itself. This will require using the Describe software as well as a USB to TTL
// cable. 
// http://www.dimensionengineering.com/info/describe
KangarooChannel KR1(K1, '1', 129); // used to be '3' and 128
KangarooChannel KR2(K1, '2', 129); // used to be '4' and 128

KangarooChannel KF1(K2, '1', 128);
KangarooChannel KF2(K2, '2', 128);

// RC mappings -- strafe: aileron, drive: elevation, turn: rudder
int strafePinRC = 3, drivePinRC = 2, turnPinRC = 4;
int strafeSignal5Vpin = 6;
int eStopPin = 5;
// note: sabertooth pins are 6 for Tx(S1) and 7 for EStop(S2)
// *********************
// RC Vars
// *********************
unsigned long DRIVE_PULSE_WIDTH;
unsigned long TURN_PULSE_WIDTH;
unsigned long STRAFE_PULSE_WIDTH;
float pulseLow = 1051, pulseHigh = 1890;

float mByte = 0, bByte = 0;
float mFloat = 0, bFloat = 0;

//connection loss timeout
volatile int rxTimeoutCounter = 0;
int rxTimeoutTime = 10;  //1 second timeout on 10 Hz timer

//track input source
enum OPERATION_STATES{ WIFI = 0, RC = 1 };
OPERATION_STATES operationState = WIFI, prev_operationState = RC;


// ****************************************************
// Initial setup function, called once
// RETURNS: none
// ****************************************************
void setup() {
  Serial.begin(115200);  //debug output for teensy controller and also input for USB controls sent from Pi

  SerialPort1.begin(9600);   // . // //temporarily ganged both drivers for debugging purposes. 

  //SerialPort1.setTimeout(50);

  SerialPort2.begin(9600);   //receiving port from RPi temporarily for debug purposes 
  //Serial.listen();      //not sure why this listen command is commented out
  // the most reliable baud rate to the kangaroo according to SuperDroid
  SerialPort2.setTimeout(50);
                          // kangaroos are default at 9600
   Serial.print("Serial comms opened");

  // Start each Kangaroo channel. The commented ".wait()" command
  // holds the program until init has completed. This is not necessary
  KR1.start();
  KR1.home();//.wait();

  KR2.start();
  KR2.home();//.wait();
  
  KF1.start();
  KF1.home();//.wait();

  KF2.start();
  KF2.home();//.wait();
 
 Serial.print("Motor drivers started up");

//K1.serialTimeout(1000); // If we don't send anything to the Kangaroo for 1 second (1000 ms),
                          // it will abort and hold position (if the last command was position)
                          // or hold velocity (if the last command was velocity).
                          // Since serial timeout is intended as a form of safety abort, to
                          // recover from a serial timeout, start() has to be issued again.
                          // Try disconnecting the TX line from S1 temporarily to see it in action.

  // set estop pin as input and pull high
  pinMode(eStopPin,INPUT); 

 // Set our input pins for RF as such 
  pinMode(turnPinRC, INPUT); 
  pinMode(drivePinRC, INPUT);
  pinMode(strafePinRC, INPUT);
  
  // 3.3V or 5V reference for strafe signal (add level shifter for 5V if receiver needs this)
  pinMode(strafeSignal5Vpin, OUTPUT); 
  digitalWrite(strafeSignal5Vpin, HIGH);
  
  // slope/intercept for converting RC signal to range [-1,1]
  mFloat = (float)2 / (pulseHigh - pulseLow);
  bFloat = -1*pulseLow*mFloat;
  
  // slope/intercept for converting [-1,1] to [-127,127]
  mByte = (float)255 / (1 -  0);
  bByte = 0;


}

//create globals for RF values
 float driveVal =  0;
 float turnVal = 0;
 float strafeVal = 0;

//create globals to store most recent wifi data
 float driveWifiVal = 0;
 float turnWifiVal = 0;
 float strafeWifiVal = 0;

//create globals to store wifi stuff
 int mappedmotorFR = 0; //these spinning backwards
 int mappedmotorFL = 0; //FL
 int mappedmotorRR = 0; //these spinning backwards //RL
 int mappedmotorRL = 0; //

// const size_t bufferSize = JSON_OBJECT_SIZE(3) + 50; //from https://bblanchon.github.io/ArduinoJson/assistant/


// ****************************************************
// Main program loop. We'll cycle through commands here
// RETURNS: none
// ****************************************************
void loop() {
  rxTimeoutCounter++;
  if (rxTimeoutCounter > 100) { rxTimeoutCounter = 100; }
  // Switch control source based on operation state //
  OPERATION_STATES next_operationState = operationState;
  switch (operationState) {
    case WIFI: {
      //-- ENTRY --//
      if (prev_operationState != WIFI) { //initializes as RC
        Serial.println("Entered WIFI state");
        powerOff();
      }

      //-- ACTIONS --//
      
      // Check if we're receiving packets
      bool packetReceived = getWifiData(); //getWifiData returns true if data is collected, this isn't set up
      if (packetReceived) {
        rxTimeoutCounter = 0;
        //debugPrintRxPacket();
      }

      // Check for rx timeout
      checkRxTimeout();

      // Command Motors
      //commandMotorsWifi();
 
      //-- TRANSITIONS --//
      // Check RC MODE input
      DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH);//, PULSEIN_TIMEOUT); //  if width > 1500 it moves forward
      TURN_PULSE_WIDTH  = pulseIn(turnPinRC, HIGH);//, PULSEIN_TIMEOUT); //if width > 1500 it rotates right CW
      STRAFE_PULSE_WIDTH  = pulseIn(strafePinRC, HIGH);//, PULSEIN_TIMEOUT); // if width > 1500 it shifts right

    //  // If pulses detected, then there is an RC signal so set state to RC
      if(DRIVE_PULSE_WIDTH > 500 && TURN_PULSE_WIDTH > 500 && STRAFE_PULSE_WIDTH > 500) {
        Serial.print("RF Signal detected");
        Serial.println();
        powerOff(); //turn off motors
        next_operationState = RC; }
        
      break;      
    }
    case RC: {
      //-- ENTRY --//
      if (prev_operationState != RC) {
        Serial.println("Entered RC state");
        powerOff();
      }    

      //-- ACTIONS --//
 
     // Read in the RC pulses
     ////~1500 is 0 mark 
    
      DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH);//, PULSEIN_TIMEOUT); //  if width > 1500 it moves forward
      TURN_PULSE_WIDTH  = pulseIn(turnPinRC, HIGH);//, PULSEIN_TIMEOUT); //if width > 1500 it rotates right CW
      STRAFE_PULSE_WIDTH  = pulseIn(strafePinRC, HIGH);//, PULSEIN_TIMEOUT); // if width > 1500 it shifts right
    
    //  // If pulses too short, throw sabertooth estop
      if(DRIVE_PULSE_WIDTH < 500 || TURN_PULSE_WIDTH < 500 || STRAFE_PULSE_WIDTH < 500) {
        //digitalWrite(eStopPin, LOW);
        Serial.print("RF Signal is bad or missing");
        Serial.println();
        powerOff(); //turn off motors
              //-- TRANSITIONS --//
        next_operationState = WIFI;  
        break;
        }
    
      // convert RC signals to continuous values from [-1,1]
      float driveVal = convertRCtoFloat(DRIVE_PULSE_WIDTH);
      float strafeVal  = -1*convertRCtoFloat(TURN_PULSE_WIDTH);
      float turnVal = convertRCtoFloat(STRAFE_PULSE_WIDTH);
    //  
      // convert the [-1,1] values to bytes in range [-127,127] for sabertooths
      //this also appears to be mixing the values in order to drive each wheel correctly
      // I checked the output and it works, brilliant I dont knnow how.
      // I'm going to try casting as a different type cause chars 
    //  char motorFR = -1*convertFloatToByte(driveVal + turnVal + strafeVal);
    //  char motorRR = convertFloatToByte(driveVal + turnVal - strafeVal);
    //  char motorFL = -1*convertFloatToByte(driveVal - turnVal - strafeVal);
    //  char motorRL = convertFloatToByte(driveVal - turnVal + strafeVal);
    
      int motorFR = -1*convertFloatToByte(driveVal + turnVal + strafeVal);
      int motorRR = convertFloatToByte(driveVal + turnVal - strafeVal);
      int motorFL = -1*convertFloatToByte(driveVal - turnVal - strafeVal);
      int motorRL = convertFloatToByte(driveVal - turnVal + strafeVal);  
    
    
      int mappedmotorFR = map(motorFR, -127, 127, 300, -300); //these spinning backwards
      int mappedmotorFL = map(motorFL, -127, 127, -300, 300); //FL
      int mappedmotorRR = map(motorRR, -127, 127, -300, 300); //these spinning backwards //RL
      int mappedmotorRL = map(motorRL, -127, 127, 300, -300); //
      
    //  int motorFR = -1*(driveVal + turnVal + strafeVal);
    //  int motorRR = (driveVal + turnVal - strafeVal);
    //  int motorFL = -1*(driveVal - turnVal - strafeVal);
    //  int motorRL = (driveVal - turnVal + strafeVal); 
    
    // command motors for kangaroo drivers
        KF1.s(mappedmotorFL); //motor '1'
        KF2.s(mappedmotorFR); //motor '2'   
        KR1.s(mappedmotorRL); //motor '3'
        KR2.s(mappedmotorRR); //motor '4'
      
      //mcSerial.print("EOF");  //realterm sync
      
      // debug print
    //  Serial.print("RF Drive: ");
    //  Serial.print(DRIVE_PULSE_WIDTH); 
    //  Serial.println(""); 
    //  Serial.print("RF Strafe: ");
    //  Serial.print(STRAFE_PULSE_WIDTH);
    //  Serial.println(",");  
    //  Serial.print("RF Turn: ");
    //  Serial.print(TURN_PULSE_WIDTH);
    //  Serial.println(","); 
    //
    //
    //  Serial.print(motorFR);  
    //  Serial.print(","); 
    //  Serial.print(motorRR);
    //  Serial.print(",");
    //  Serial.print(motorFL);  
    //  Serial.print(","); 
    //  Serial.print(motorRL);  
    //  Serial.print(","); 
    //
    //  Serial.print(driveVal);  
    //  Serial.print(","); 
    //  Serial.print(turnVal);
    //  Serial.print(",");
    //  Serial.print(strafeVal);  
    //  Serial.print(","); 
    


    }
  }
  prev_operationState = operationState;
  operationState = next_operationState;
}

//void commandMotorsWifi(void){
//    // command motors for kangaroo drivers
//    KF1.s(mappedmotorFL); //motor '1'
//    KF2.s(mappedmotorFR); //motor '2'   
//    KR1.s(mappedmotorRL); //motor '3'
//    KR2.s(mappedmotorRR); //motor '4'
//}
  

void powerOff(void){
    //this function can be used to save power when bot is not being driven
    // Notice that the Kangaroo provides no special resistance while the channel is powered down.
  KF1.powerDown();
  KF2.powerDown();
  KR1.powerDown();
  KR2.powerDown();
  }
void checkRxTimeout(void)
{
  if (rxTimeoutCounter >= rxTimeoutTime) { powerOff(); }
  Serial.println();
  Serial.print("rXtimeout");
  Serial.println();
  return;
}

bool getWifiData(void){
    char inByte = ' ';
    
    if (SerialPort2.available()){  
    char inByte = SerialPort2.read();
    Serial.print("Print the Inbyte");
    Serial.println(inByte);
    
    ////consider increasing buffer to 1024...
    //  StaticJsonBuffer<20000> jsonBuffer;
    //
    //  JsonObject& root = jsonBuffer.parseObject(SerialPort2);
    //
    //  // Test if parsing succeeds.
    //  if (!root.success()) {
    //    Serial.println("parseObject() failed");
    //    return false;
    //  }
    
       //false values to send over serial {"drive":0.5,"strafe":0.3,"turn":0.2}
      //bind parsed dictionary keys to local variables
    //  float driveWifiVal = root["d"];
    //  float turnWifiVal = root["t"];
    //  float strafeWifiVal = root["s"];
     
      int motorFR = -1*convertFloatToByte(driveVal + turnVal + strafeVal);
      int motorRR = convertFloatToByte(driveVal + turnVal - strafeVal);
      int motorFL = -1*convertFloatToByte(driveVal - turnVal - strafeVal);
      int motorRL = convertFloatToByte(driveVal - turnVal + strafeVal);  
      
      
      int mappedmotorFR = map(motorFR, -127, 127, 300, -300); //these spinning backwards
      int mappedmotorFL = map(motorFL, -127, 127, -300, 300); //FL
      int mappedmotorRR = map(motorRR, -127, 127, -300, 300); //these spinning backwards //RL
      int mappedmotorRL = map(motorRL, -127, 127, 300, -300); //
      
      //  int motorFR = -1*(driveVal + turnVal + strafeVal);
      //  int motorRR = (driveVal + turnVal - strafeVal);
      //  int motorFL = -1*(driveVal - turnVal - strafeVal);
      //  int motorRL = (driveVal - turnVal + strafeVal); 
      
      // command motors for kangaroo drivers
        KF1.s(mappedmotorFL); //motor '1'
        KF2.s(mappedmotorFR); //motor '2'   
        KR1.s(mappedmotorRL); //motor '3'
        KR2.s(mappedmotorRR); //motor '4'
    //DEBUG print variables
      Serial.println();
      Serial.print("Drive WIFI:");
      Serial.print(driveWifiVal);
      Serial.println();
      Serial.print("Turn WIFI:");
      Serial.print(turnWifiVal);
      Serial.println();
      Serial.print("Strafe WIFI:");
      Serial.print(strafeWifiVal);
      Serial.println();
      Serial.print("Motor FR:");
      Serial.print(motorFR);
      Serial.println();
      Serial.print("Motor FL");
      Serial.print(motorFL);
      Serial.println();
      Serial.print("Mapped Motor FR:");
      Serial.print(mappedmotorFR);
      Serial.println();
        
    //if (Serial.available() > 0) {
    //                // read the incoming byte:
    //  { inData = Serial.readStringUntil('\n');
    //    Serial.println("data: "+inData);
    //
    //     inData = ""; //clears buffer
    //  }
    //}
      return true;
     }
}
  
float convertRCtoFloat(unsigned long pulseWidth)
{
  // deadband - to increase the deadband adjust first value (1450) down and second value (1550) up
  if(pulseWidth > 1450 && pulseWidth < 1550) { pulseWidth = (float)(pulseHigh + pulseLow) / 2; }
  
  float checkVal = mFloat*pulseWidth + bFloat - 1;
  checkVal = checkVal < -1 ? -1 : checkVal;
  checkVal = checkVal >  1 ?  1 : checkVal;
//  Serial.print(checkVal);
  return checkVal;
}

int convertFloatToByte(float value)
{
  //converts from range -1,1 to range -127 to 127
  float checkVal = mByte*value + bByte; // y = mx + b 
  checkVal = checkVal < -127 ? -127 : checkVal; //sets a lower limit on what the value can be
  checkVal = checkVal >  127 ?  127 : checkVal; // sets an upper limit
//  Serial.print(checkVal);
//  Serial.print(" ");

//  char returnVal = (char)(checkVal);
//  return returnVal;
  return checkVal;
}


