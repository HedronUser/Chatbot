// ****************************************************
// Libraries
// ****************************************************
#include <Kangaroo.h>
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3

// mode options
const int modePrintFwdBack = 0;
const int modeSerialDelayTest = 1;

// Set the current testing mode to one of the options declared above
int mode = modeSerialDelayTest;

// If sending takes over this amount of time, report it in serial delay test mode
long DELAYED_SEND_THRESHOLD_MILLIS = 40;

int baud = 9600;

// ****************************************************
// Motor Controller Initialization
// ****************************************************
//truths
// white wire is S1 front
// blue wire is S2 front

//white wire is S1 rear
//yellow wire is S2 rear

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
KangarooChannel KR1(K1, '1', 129); // used to be '3'
KangarooChannel KR2(K1, '2', 129); // used to be '4'

KangarooChannel KF1(K2, '1', 128);
KangarooChannel KF2(K2, '2', 128);

int strafeSignal5Vpin = 6;
int eStopPin = 5;

float mByte = 0, bByte = 0;
float mFloat = 0, bFloat = 0;

bool isForward = true;

// ****************************************************
// Initial setup function, called once
// RETURNS: none
// ****************************************************
void setup() {
  Serial.begin(baud);  //debug output for teensy controller and also input for USB controls sent from Pi
  SerialPort1.begin(baud);   // Initialize our Serial to 115200. This seems to be
  SerialPort2.begin(baud);   // Initialize our Serial to 115200. This seems to be

  //Serial.listen();      //not sure why this listen command is commented out
  // the most reliable baud rate to the kangaroo according to SuperDroid
                          // kangaroos are default at 9600
   Serial.print("getting here");

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
 Serial.print("getting here");

//K1.serialTimeout(1000); // If we don't send anything to the Kangaroo for 1 second (1000 ms),
                          // it will abort and hold position (if the last command was position)
                          // or hold velocity (if the last command was velocity).
                          // Since serial timeout is intended as a form of safety abort, to
                          // recover from a serial timeout, start() has to be issued again.
                          // Try disconnecting the TX line from S1 temporarily to see it in action.

  // set estop pin as input and pull high
  pinMode(eStopPin,INPUT); 
  
  // 3.3V or 5V reference for strafe signal (add level shifter for 5V if receiver needs this)
  pinMode(strafeSignal5Vpin, OUTPUT); 
  digitalWrite(strafeSignal5Vpin, HIGH);

}

// ****************************************************
// Main program loop. We'll cycle through commands here
// RETURNS: none
// ****************************************************
void loop() {

    switch (mode) {
        case modePrintFwdBack:
          handlePrintFwdBackTest();
          break;
        case modeSerialDelayTest:
          handleSerialDelayTest();
          break;
        default: 
          Serial.print("Invalid test mode: "); Serial.println(mode);
          break;
    }
}

void handlePrintFwdBackTest() {

    int val;
    String fwdBackMsg;
    if (isForward) {
        val = 300;
        fwdBackMsg = "forward";
    } else {
        val = -300;
        fwdBackMsg = "backward";
    }

    long totalStartTime = millis();

    // send 1
    long beforeSendTime = millis();
    Serial.println("start send 1");
    KF1.s(val); //motor '1'    
    Serial.print("Send 1 took: "); Serial.println(millis()-beforeSendTime);

    // send 2
    beforeSendTime = millis();
    Serial.println("start send 2");
    KF2.s(val); //motor '2'   
    Serial.print("Send 2 took: "); Serial.println(millis()-beforeSendTime);

    // send 3
    beforeSendTime = millis();
    Serial.println("start send 3");
    KR1.s(val); //motor '3'
    Serial.print("Send 3 took: "); Serial.println(millis()-beforeSendTime);

    // send 4
    beforeSendTime = millis();
    Serial.println("start send 4");
    KR2.s(val); //motor '4'
    Serial.print("Send 4 took: "); Serial.println(millis()-beforeSendTime);

    long totalDuration = millis() - totalStartTime;

    Serial.print("Sent all four "); Serial.print(fwdBackMsg); Serial.print(" in: "); Serial.print(totalDuration);Serial.println("\n\n------------------------\n\n");

    
    // toggle drive direction
    isForward = !isForward;

    delay(1500);
}

void handleSerialDelayTest() {
   
    int val = random(-300, 301);

    // send 1
    long beforeSendTime = millis();
    KF1.s(val); //motor '1' 
    handleDelayedSendReport(beforeSendTime, "1");   

    // send 2
    beforeSendTime = millis();
    KF2.s(val); //motor '2'   
    handleDelayedSendReport(beforeSendTime, "2"); 

    // send 3
    beforeSendTime = millis();
    KR1.s(val); //motor '3'
    handleDelayedSendReport(beforeSendTime, "3"); 

    // send 4
    beforeSendTime = millis();
    KR2.s(val); //motor '4'
    handleDelayedSendReport(beforeSendTime, "4");

    // short delay only for this test
    delay(10);
}

void handleDelayedSendReport(long beforeSendTime, String motorID) {
    long duration = millis() - beforeSendTime;
    // If the duration was over the threshold, report it
    if (duration >= DELAYED_SEND_THRESHOLD_MILLIS) {
        Serial.print("Detected delayed send to motor "); Serial.print(motorID); Serial.print(" of: "); Serial.println(duration);
        Serial.println("\n--------------------------\n");
    }
}


