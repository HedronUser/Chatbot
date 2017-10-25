//Serial Comm based on ArduinoPC2, published here http://forum.arduino.cc/index.php?topic=225329.0
//
//http://forum.arduino.cc/index.php?topic=225329.0

#include <Kangaroo.h>
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3
//#include <digitalWriteFast.h>

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
// RESPONSE_VARIABLES
// *********************
// If true, will respond on every loop
// Otherwise, will respond only when control commands are issued
bool IS_VERBOSE_RESPONSE = true;

// *********************
// CONTROL_VARIABLES
// *********************
// Ignore control signals inside the dead band
float CONTROL_DEAD_BAND_MIN = -0.1;
float CONTROL_DEAD_BAND_MAX = 0.1;

// Store the last-received control values
float driveVal = 0;
float strafeVal = 0;
float turnVal = 0;

// If data isn't received for this amount of time, will power off the motors
unsigned long NO_DATA_CONNECTION_TIME_MILLIS = 1000;
unsigned long lastDataReceivedTime;
// Is there currently a data connection? Will be set false if NO_DATA_CONNECTION_TIME_MILLIS passes without receiving data
bool hasDataConnection = true;

// *********************
// RC Vars
// *********************
float RC_PULSE_LOW = 1051, RC_PULSE_HIGH = 1890;
// If RC pulse values are all below this minimum, there is no RC control present
unsigned long RC_ACTIVE_PULSE_MINIMUM = 500;
unsigned long RC_PULSEIN_TIMEOUT_MICROS = 20000; //was 30000

float mByte = 0, bByte = 0;
float mFloat = 0, bFloat = 0;

unsigned long rcDrivePulseWidth;
unsigned long rcTurnPulseWidth;
unsigned long rcStrafePulseWidth;

// RC mappings -- strafe: aileron, drive: elevation, turn: rudder
int turnPinRC = 3, drivePinRC = 2, strafePinRC = 4;
int strafeSignal5Vpin = 6;

// *********************
// E-Stop Button Vars
// *********************
const int eStopPin = 12;
bool isEstop = false;

// ****************************************************
// Wifi Variables
// ****************************************************
//connection loss timeout
volatile int rxTimeoutCounter = 0;
int rxTimeoutTime = 10;  //1 second timeout on 10 Hz timer

//Not goi
//track input source
enum OPERATION_STATES{ WIFI = 0, RC = 1 };
OPERATION_STATES operationState = RC;

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
boolean newData = false;

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

void setup() {
  Serial.begin(115200); //comms to pi
  SerialPort1.begin(115200);   // . // //temporarily ganged both drivers for debugging purposes. 
  SerialPort2.begin(115200);   // going to hook back up to the motor drivers
    // flash LEDs so we know we are alive
  for (byte n = 0; n < numLEDs; n++) {
     pinMode(ledPin[n], OUTPUT);
     digitalWriteFast(ledPin[n], HIGH);
  }
  
  delay(5000); // delay() is OK in setup as it only happens once
  
  for (byte n = 0; n < numLEDs; n++) {
     digitalWriteFast(ledPin[n], LOW);
  }
  
    // tell the PC we are ready, this is necessary, basically we are plugging in the USB and have 5 seconds before this gets thrown

  handleKangarooSetup();

   // set estop pin as input and pull high
  pinMode(eStopPin,INPUT_PULLUP); //this is hooked to a NC Estop switch with the other side of estop switch hooked to ground.

 // Set our input pins for RC as such 
  pinMode(turnPinRC, INPUT); 
  pinMode(drivePinRC, INPUT);
  pinMode(strafePinRC, INPUT);
  
  // 3.3V or 5V reference for strafe signal (add level shifter for 5V if receiver needs this)
  pinMode(strafeSignal5Vpin, OUTPUT); 
  digitalWrite(strafeSignal5Vpin, HIGH);
  
  // slope/intercept for converting RC signal to range [-1,1]
  mFloat = (float)2 / (RC_PULSE_HIGH - RC_PULSE_LOW);
  bFloat = -1*RC_PULSE_LOW*mFloat;
  
  // slope/intercept for converting [-1,1] to [-127,127]
  mByte = (float)255 / (1 -  0);
  bByte = 0;

  // Init last signal time to now to prevent the teensy from automatically shutting down the motors
  lastDataReceivedTime = millis();
  
  sendMsgToPC("Arduino is ready");

}

//=============

void loop() {
  curMillis = millis();
    
  // Handle operation mode update between WIFI/RC
  updateOperationState();

  // handles new messages and sets the hasDataConnection state
  updateData();  

  // In verbose mode, log every loop
  if (IS_VERBOSE_RESPONSE && newData) {
    replyToPC();  
  }
  
  //flashLEDs();

  // update the estop - if estop starts, will power off the kangaroos
  updateEstop();

  /* Commands motors and replys to PC on fixed interval if:
        not in Estop
            AND
        there's new data
  */
  if (!isEstop && newData && (curMillis - prevReplyToPCmillis) > replyToPCinterval) {
        replyToPC();
        commandMotors(driveVal, turnVal, strafeVal);
        counter = counter + 1;
        prevReplyToPCmillis = curMillis;
  }
  
  // Reset this flag, since any new data has been handled
  newData = false;
  unsigned long afterRead = millis();
  unsigned long timeElapsed = afterRead -  curMillis;
  
  //  Serial.print("Time it takes thru loop: ");
  //  Serial.println(timeElapsed);
}

void handleKangarooSetup(void){
  
  KR1.start();
  KR1.home();//.wait();

  KR2.start();
  KR2.home();//.wait();
  
  KF1.start();
  KF1.home();//.wait();

  KF2.start();
  KF2.home();//.wait(); 
}

//============

void updateEstop(void){
  bool wasEstop = isEstop;
  isEstop = digitalReadFast(eStopPin);

  if (wasEstop && !isEstop && hasDataConnection) {
    // estop just ended AND there's a data connection - so power up the kangaroos
    handleKangarooSetup();
    sendMsgToPC("Estop ended - powering up kangaroos");
  } else if (isEstop && !wasEstop) {
    // estop just started - send a powerOff()
    powerOff();
    sendMsgToPC("Estop started - powering off kangaroos");
  }
}

void updateOperationState() {
    // check if there's an RC signal present - updates the RC pulse widths if RC is present
    bool hasRCSignalNow = updateRCSignal();
    
    if (operationState == RC) {
        if (!hasRCSignalNow) {
            sendMsgToPC("RC signal lost - powering off and switching to WIFI control mode");
            powerOff(); // turn off motors
            operationState = WIFI;
        }
    } else {
        // operation state WIFI
        if (hasRCSignalNow) {
            sendMsgToPC("RC signal detected - switching to RC control");
            operationState = RC;
        }
    }
}

void updateData() {
  // In wifi mode, update the variables from the PC
  if (operationState == WIFI) {
    getDataFromPC();
    //set flag for update when buffer is full
    updateVariable();
  } else {
    getDataFromRC();
  }

  // ---- Handle power-off/power-on if data stream is disconnected/reconnected ---- //
  bool wasConnected = hasDataConnection;
  hasDataConnection = hasCurrentDataConnection();

  if (wasConnected && !hasDataConnection) {
    // lost connection - power down
    powerOff();
    // Will send this message to the PC, even though it's probably not connected
    sendMsgToPC("Lost data connection - powering down kangaroos");
  } else if (!wasConnected && hasDataConnection && !isEstop) {
    // Just reinstated connection AND not in estop, will start up the motors
    handleKangarooSetup();
    sendMsgToPC("Regained data connection connection - powering up kangaroos");
  }
}

/**
 * Return true if the last data was received within the data connection time threshold
 */
bool hasCurrentDataConnection() {
    return millis() - lastDataReceivedTime < NO_DATA_CONNECTION_TIME_MILLIS;
}

/**
 * Updates the first RC pulse width to check if there's a signal
 * If there's no signal, returns false immediately to reduce latency
 * If there is a signal, updates the remaining pulse width values and returns true
 */
bool updateRCSignal() {

    rcDrivePulseWidth = pulseIn(drivePinRC, HIGH, RC_PULSEIN_TIMEOUT_MICROS);
    if (rcDrivePulseWidth < RC_ACTIVE_PULSE_MINIMUM) {
        // there's no signal - return false immediately to reduce latency
        return false;
    }

    // If we make it this far, there is a signal, so update the other pulse width values
    rcTurnPulseWidth  = pulseIn(turnPinRC, HIGH, RC_PULSEIN_TIMEOUT_MICROS);
    rcStrafePulseWidth  = pulseIn(strafePinRC, HIGH, RC_PULSEIN_TIMEOUT_MICROS);

    return true;
    
}

void commandMotors(float driveVal, float turnVal, float strafeVal){
//-- ACTIONS --//

    float filteredDrive = deadBandFilter(driveVal);
    float filteredTurn = deadBandFilter(turnVal);
    float filteredStrafe = deadBandFilter(strafeVal);   
  
    int motorFR = -1*convertFloatToByte(filteredDrive - filteredTurn - filteredStrafe);
    int motorRR = convertFloatToByte(filteredDrive - filteredTurn + filteredStrafe);
    int motorFL = -1*convertFloatToByte(filteredDrive + filteredTurn + filteredStrafe);
    int motorRL = convertFloatToByte(filteredDrive + filteredTurn - filteredStrafe);  

    // if we pass true, the mapping will be flipped to prevent backwards spinning
    int mappedmotorFR = mapMotorValue(motorFR, true);
    int mappedmotorFL = mapMotorValue(motorFL, false);
    int mappedmotorRR = mapMotorValue(motorRR, false);
    int mappedmotorRL = mapMotorValue(motorRL, true);

    /*sendMsgToPC("---- Motor commands sent ----");
    sendMsgToPC("FR: " + getFourDigit(mappedmotorFR));  
    sendMsgToPC("\tRR: " + getFourDigit(mappedmotorRR));
    sendMsgToPC("\tFL: " + getFourDigit(mappedmotorFL));  
    sendMsgToPC("\tRL: " + getFourDigit(mappedmotorRL));
    sendMsgToPC("");*/
      
      // command motors for kangaroo drivers
    KF1.s(mappedmotorFL); //motor '1'
    KF2.s(mappedmotorFR); //motor '2'   
    KR1.s(mappedmotorRL); //motor '3'
    KR2.s(mappedmotorRR); //motor '4'

}

//=============

void getDataFromPC() {

  // receive data from PC and save it into inputBuffer
    
  while (Serial.available() > 0) {

    char x = Serial.read();

    // the order of these IF clauses is significant
      
    if (readInProgress && x == endMarker) {
      readInProgress = false;
      handleNewData();
      inputBuffer[bytesRecvd] = 0;
      parseData();
    }
    
    if (readInProgress) {
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
  
  // don't try reading across update loops
  readInProgress = false;
}

// ===========
void getDataFromRC() {
  driveVal = convertRCtoFloat(rcDrivePulseWidth);
  strafeVal  = -1*convertRCtoFloat(rcStrafePulseWidth);
  turnVal = convertRCtoFloat(rcTurnPulseWidth);
  handleNewData();
}

/*
 * Sets the newdata flag true, and sets the time of last signal received
 */
void handleNewData() {
    newData = true;
    lastDataReceivedTime = millis();
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

// Sensds msg to pc with start and end characters
void sendMsgToPC(String msg) {
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
       digitalWriteFast( ledPin[n], ! digitalReadFast( ledPin[n]) );
    }
  }
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
  if(pulseWidth > 1450 && pulseWidth < 1550) { pulseWidth = (float)(RC_PULSE_HIGH + RC_PULSE_LOW) / 2; }
  
  float checkVal = mFloat*pulseWidth + bFloat - 1;
  checkVal = checkVal < -1 ? -1 : checkVal;
  checkVal = checkVal >  1 ?  1 : checkVal;
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

//  char returnVal = (char)(checkVal);
//  return returnVal;
  return checkVal;
}

/**
 * All values range from -127 to +127
 * Some map to -300 to + 300, while others map to 300 to -300 to prevent backwards spinning
 * If isFlipMap is true, then it'll map -127 to +300 and vice versa
 */
int mapMotorValue(int value, bool isFlipMap) {

    // Don't run 0 values through the map, since this can result in noise
    if (value == 0) {
        return 0;
    }

    // flip the mapping if the flag is set
    int minMap = isFlipMap ? 300 : -300;
    int maxMap = isFlipMap ? -300 : 300;

    return map(value, -127, 127, minMap, maxMap);
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

// get val with leading 0s
String getFourDigit(int val) 
{
    String fourDigit = "";
    if (val < 0) fourDigit += '-';
    int absVal = abs(val);
    if (absVal < 1000) fourDigit += '0';
    if (absVal < 100) fourDigit += '0';
    if (absVal < 10) fourDigit += '0';
    fourDigit += absVal;
    return fourDigit;
}

