

// ****************************************************
// Libraries
// ****************************************************
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3

// toggle this flag to include/exclude the motor command prints
bool IS_PRINT_MOTOR_COMMANDS = true;

// *********************
// RC Vars
// *********************
// RC mappings -- strafe: aileron, drive: elevation, turn: rudder
int strafePinRC = 3, drivePinRC = 2, turnPinRC = 4;

unsigned long DRIVE_PULSE_WIDTH;
unsigned long TURN_PULSE_WIDTH;
unsigned long STRAFE_PULSE_WIDTH;
float pulseLow = 1051, pulseHigh = 1890;

unsigned long RC_PULSEIN_TIMEOUT_MICROS = 15000;
bool USE_RC_TIMEOUT = true;

float mByte = 0, bByte = 0;
float mFloat = 0, bFloat = 0;

// ****************************************************
// Initial setup function, called once
// RETURNS: none
// ****************************************************
void setup() {
  Serial.begin(9600);  //debug output for teensy controller and also input for USB controls sent from Pi

 // Set our input pins for RF as such 
  pinMode(turnPinRC, INPUT); 
  pinMode(drivePinRC, INPUT);
  pinMode(strafePinRC, INPUT);
  
  // slope/intercept for converting RC signal to range [-1,1]
  mFloat = (float)2 / (pulseHigh - pulseLow);
  bFloat = -1*pulseLow*mFloat;
  
  // slope/intercept for converting [-1,1] to [-127,127]
  mByte = (float)255 / (1 -  0);
  bByte = 0;
}

// ****************************************************
// Main program loop. We'll cycle through commands here
// RETURNS: none
// ****************************************************
void loop() {

 if (USE_RC_TIMEOUT) {
    long before = millis();
    DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH, RC_PULSEIN_TIMEOUT_MICROS);
    TURN_PULSE_WIDTH  = pulseIn(turnPinRC, HIGH, RC_PULSEIN_TIMEOUT_MICROS);
    STRAFE_PULSE_WIDTH  = pulseIn(strafePinRC, HIGH, RC_PULSEIN_TIMEOUT_MICROS);
    Serial.print("RC pulseIn with timeout took: "); Serial.println(millis()-before);
 } else {
    long before = millis();
     // Read in the RC pulses
    DRIVE_PULSE_WIDTH = pulseIn(drivePinRC, HIGH);
    TURN_PULSE_WIDTH  = pulseIn(turnPinRC, HIGH);
    STRAFE_PULSE_WIDTH  = pulseIn(strafePinRC, HIGH);
    Serial.print("RC pulseIn without timeout took: "); Serial.println(millis()-before);
 }

//  // If pulses too short, throw sabertooth estop
  if(DRIVE_PULSE_WIDTH < 500 || TURN_PULSE_WIDTH < 500 || STRAFE_PULSE_WIDTH < 500) {
    Serial.println("RC Signal is bad or missing");
    return;
  }

  // convert RC signals to continuous values from [-1,1]
  float driveVal = convertRCtoFloat(DRIVE_PULSE_WIDTH);
  float strafeVal  = -1*convertRCtoFloat(TURN_PULSE_WIDTH);
  float turnVal = convertRCtoFloat(STRAFE_PULSE_WIDTH);

  // debug print
  Serial.print("Drive: ");      printFourDigit(DRIVE_PULSE_WIDTH); 
  Serial.print("\tStrafe: ");   printFourDigit(STRAFE_PULSE_WIDTH); 
  Serial.print("\tTurn: ");     printFourDigit(TURN_PULSE_WIDTH);
  Serial.println(""); 

  if (IS_PRINT_MOTOR_COMMANDS) {
    int motorFR = -1*convertFloatToByte(driveVal + turnVal + strafeVal);
    int motorRR = convertFloatToByte(driveVal + turnVal - strafeVal);
    int motorFL = -1*convertFloatToByte(driveVal - turnVal - strafeVal);
    int motorRL = convertFloatToByte(driveVal - turnVal + strafeVal);  

    int mappedmotorFR = map(motorFR, -127, 127, 300, -300); //these spinning backwards
    int mappedmotorFL = map(motorFL, -127, 127, -300, 300); //FL
    int mappedmotorRR = map(motorRR, -127, 127, -300, 300); //these spinning backwards //RL
    int mappedmotorRL = map(motorRL, -127, 127, 300, -300); //

    Serial.print("FR: ");   printFourDigit(mappedmotorFR);  
    Serial.print("\tRR: "); printFourDigit(mappedmotorRR);
    Serial.print("\tFL: "); printFourDigit(mappedmotorFL);  
    Serial.print("\tRL: "); printFourDigit(mappedmotorRL);
    Serial.println("");
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
  float checkVal = mByte*value + bByte; // y = mx + b 
  checkVal = checkVal < -127 ? -127 : checkVal; //sets a lower limit on what the value can be
  checkVal = checkVal >  127 ?  127 : checkVal; // sets an upper limit
//  Serial.print(checkVal);
//  Serial.print(" ");

//  char returnVal = (char)(checkVal);
    return checkVal;
}

// Print leading 0s
void printFourDigit(int val) 
{
    if (val < 0) Serial.print('-');
    int absVal = abs(val);
    if (absVal < 1000) Serial.print('0');
    if (absVal < 100) Serial.print('0');
    if (absVal < 10) Serial.print('0');
    Serial.print(absVal);
}


