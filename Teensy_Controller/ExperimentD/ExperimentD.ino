//Serial Comm based on ArduinoPC2, published here http://forum.arduino.cc/index.php?topic=225329.0
//
//http://forum.arduino.cc/index.php?topic=225329.0

#include <Kangaroo.h>
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3

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

KangarooChannel KR1(K1, '1', 129); // used to be '3' and 128
KangarooChannel KR2(K1, '2', 129); // used to be '4' and 128

KangarooChannel KF1(K2, '1', 128);
KangarooChannel KF2(K2, '2', 128);

// *********************
// CONTROL_VARIABLES
// *********************
// Ignore control signals inside the dead band
float CONTROL_DEAD_BAND_MIN = -0.1;
float CONTROL_DEAD_BAND_MAX = 0.1;

float driveVal = 0;
float strafeVal = 0;
float turnVal = 0;

// *********************
// RC Vars
// *********************
unsigned long DRIVE_PULSE_WIDTH;
unsigned long TURN_PULSE_WIDTH;
unsigned long STRAFE_PULSE_WIDTH;
float pulseLow = 1051, pulseHigh = 1890;

float mByte = 0, bByte = 0;
float mFloat = 0, bFloat = 0;

// RC mappings -- strafe: aileron, drive: elevation, turn: rudder
int strafePinRC = 3, drivePinRC = 2, turnPinRC = 4;
int strafeSignal5Vpin = 6;
int eStopPin = 5;

// ****************************************************
// Wifi Variables
// ****************************************************
//connection loss timeout
volatile int rxTimeoutCounter = 0;
int rxTimeoutTime = 10;  //1 second timeout on 10 Hz timer

//Not goi
//track input source
enum OPERATION_STATES{ WIFI = 0, RC = 1 };
OPERATION_STATES operationState = WIFI, prev_operationState = RC;

// ****************************************************
// Serial Comm Variables (wifi stuff)
// ****************************************************

const byte numLEDs = 2;
byte ledPin[numLEDs] = {12, 13};
unsigned long LEDinterval[numLEDs] = {200, 400};
unsigned long prevLEDmillis[numLEDs] = {0, 0};

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;

char messageFromPC[buffSize] = {0};
int newFlashInterval = 0;
float servoFraction = 0.0; // fraction of servo range to move

//counter
int counter = 0;

//timer for checking wifi
unsigned long curMillis;

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 100; // 100ms command interval

//=============Create some globals to store wifi data



//create globals to store wifi stuff
 int mappedmotorFR = 0; //these spinning backwards
 int mappedmotorFL = 0; //FL
 int mappedmotorRR = 0; //these spinning backwards //RL
 int mappedmotorRL = 0; //


void setup() {
  Serial.begin(115200); //comms to pi
  SerialPort1.begin(115200);   // . // //temporarily ganged both drivers for debugging purposes. 
  SerialPort2.begin(115200);   // going to hook back up to the motor drivers
    // flash LEDs so we know we are alive
  for (byte n = 0; n < numLEDs; n++) {
     pinMode(ledPin[n], OUTPUT);
     digitalWrite(ledPin[n], HIGH);
  }
  
  delay(5000); // delay() is OK in setup as it only happens once
  
  for (byte n = 0; n < numLEDs; n++) {
     digitalWrite(ledPin[n], LOW);
  }
  
    // tell the PC we are ready, this is necessary, basically we are plugging in the USB and have 5 seconds before this gets thrown

  KR1.start();
  KR1.home();//.wait();

  KR2.start();
  KR2.home();//.wait();
  
  KF1.start();
  KF1.home();//.wait();

  KF2.start();
  KF2.home();//.wait();

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
  
  Serial.println("<Arduino is ready>");

}

//=============

void loop() {
  curMillis = millis();
  getDataFromPC();

  //set flag for update when buffer is full
  updateVariable();
  //could also parse data at this point

  
  replyToPC();
  flashLEDs();

//should only get called when finished fully parsing
//insert wifi/bot control code
  if((curMillis - prevReplyToPCmillis) > replyToPCinterval){
    commandMotors(driveVal, turnVal, strafeVal);
    counter = counter + 1;
    prevReplyToPCmillis = curMillis;
  }

}

void commandMotors(float driveVal, float turnVal, float strafeVal){
//-- ACTIONS --//

    float filteredDrive = deadBandFilter(driveVal);
    float filteredTurn = deadBandFilter(turnVal);
    float filteredStrafe = deadBandFilter(strafeVal);    
    
    int motorFR = -1*convertFloatToByte(filteredDrive + filteredTurn + filteredStrafe);
    int motorRR = convertFloatToByte(filteredDrive + filteredTurn - filteredStrafe);
    int motorFL = -1*convertFloatToByte(filteredDrive - filteredTurn - filteredStrafe);
    int motorRL = convertFloatToByte(filteredDrive - filteredTurn + filteredStrafe);  
    
    
    int mappedmotorFR = map(motorFR, -127, 127, 300, -300); //these spinning backwards
    int mappedmotorFL = map(motorFL, -127, 127, -300, 300); //FL
    int mappedmotorRR = map(motorRR, -127, 127, -300, 300); //these spinning backwards //RL
    int mappedmotorRL = map(motorRL, -127, 127, 300, -300); //
      
      // command motors for kangaroo drivers
    KF1.s(mappedmotorFL); //motor '1'
    KF2.s(mappedmotorFR); //motor '2'   
    KR1.s(mappedmotorRL); //motor '3'
    KR2.s(mappedmotorRR); //motor '4'
}

//=============

void getDataFromPC() {

    // receive data from PC and save it into inputBuffer
    
  if(Serial.available() > 0) {

    char x = Serial.read();

      // the order of these IF clauses is significant
      
    if (x == endMarker) {
      readInProgress = false;
      newDataFromPC = true;
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if(readInProgress) {
      inputBuffer[bytesRecvd] = x;
      bytesRecvd ++;
      if (bytesRecvd == buffSize) {
        bytesRecvd = buffSize - 1;
      }
    }

    if (x == startMarker) { 
      bytesRecvd = 0; 
      readInProgress = true;
    }
  }
}

//=============
 
void parseData() {

    // split the data into its parts
    
  char * strtokIndx; // this is used by strtok() as an index
  
  strtokIndx = strtok(inputBuffer,",");      // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  newFlashInterval = atoi(strtokIndx);     // convert this part to an integer
  
  strtokIndx = strtok(NULL, ","); 
  servoFraction = atof(strtokIndx);     // convert this part to a float

}

//=============

void replyToPC() {
//this function is called AFTER the updateFlashInterval (Legacy) updateVariable()
   
  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<Msg ");
    Serial.print(messageFromPC);
    Serial.print(" Integer Val ");
    Serial.print(newFlashInterval);
    Serial.print(" Float Val ");
    Serial.print(servoFraction);
    Serial.print(" Received Drive ");
    Serial.print(driveVal);
    Serial.print(" Received Strafe ");
    Serial.print(strafeVal);
    Serial.print(" Received Turn ");
    Serial.print(turnVal);
    Serial.print("Counter");
    Serial.print(counter);
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");
  }
}

//============

void updateVariable() {
//this could be changed
   // this illustrates using different inputs to call different functions  
  if (strcmp(messageFromPC, "drive") == 0) {
     driveVal = servoFraction;
  }
  
  if (strcmp(messageFromPC, "strafe") == 0) {
     strafeVal = servoFraction;
  }
    if (strcmp(messageFromPC, "turn") == 0) {
     turnVal = servoFraction;
  }

}

//=============

void flashLEDs() {

  for (byte n = 0; n < numLEDs; n++) {
    if (curMillis - prevLEDmillis[n] >= LEDinterval[n]) {
       prevLEDmillis[n] += LEDinterval[n];
       digitalWrite( ledPin[n], ! digitalRead( ledPin[n]) );
    }
  }
}


//=============


void checkRxTimeout(void)
//not sure we need this
{
  if (rxTimeoutCounter >= rxTimeoutTime) { powerOff(); }
  Serial.println();
  Serial.print("rXtimeout");
  Serial.println();
  return;
}

//=============

float deadBandFilter(float val)
{
    // if it's outside of the dead band, just return the value
    if (val < CONTROL_DEAD_BAND_MIN || val > CONTROL_DEAD_BAND_MAX) {
        return val;
    }

    // it's in the dead-band, so return 0
    return 0;
}

//=============

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

//=============

int convertFloatToByte(float value)
//this function has been changed to convert a float [-1,1] to int [-127, 127]
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

//=============

void powerOff(void){
    //this function can be used to save power when bot is not being driven
    // Notice that the Kangaroo provides no special resistance while the channel is powered down.
  KF1.powerDown();
  KF2.powerDown();
  KR1.powerDown();
  KR2.powerDown();
  }

  //=============


