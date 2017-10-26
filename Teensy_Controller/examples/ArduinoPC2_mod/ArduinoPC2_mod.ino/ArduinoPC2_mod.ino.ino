// 12 Mar 2014
// this works with ComArduino.py and ComArduinoA4e.rb
// this version uses a start marker 254 and an end marker of 255
//  it uses 253 as a special byte to be able to reproduce 253, 254 and 255
// it also sends data to the PC using the same system
//   if the number of bytes is 0 the PC will assume a debug string and just print it to the screen

//================
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3
#define TX3_PIN 8
#define RX3_PIN 7
SoftwareSerial  SerialPort2(RX3_PIN, TX3_PIN);

#define startMarker 254
#define endMarker 255
#define specialByte 253
#define maxMessage 16

// the program could be rewritten to use local variables instead of some of these globals
//  however globals make the code simpler
//  and simplify memory management

byte bytesRecvd = 0;
byte dataSentNum = 0; // the transmitted value of the number of bytes in the package i.e. the 2nd byte received
byte dataRecvCount = 0;


byte dataRecvd[maxMessage]; 
byte dataSend[maxMessage];  
byte tempBuffer[maxMessage];

byte dataSendCount = 0; // the number of 'real' bytes to be sent to the PC
byte dataTotalSend = 0; // the number of bytes to send to PC taking account of encoded bytes

boolean inProgress = false;
boolean startFound = false;
boolean allReceived = false;

float driveWifiVal = 0;

//================

void setup() {
  pinMode(13, OUTPUT); // the onboard LED
  SerialPort2.begin(57600);
  Serial.begin(9600);
  delay(1000);

  debugToPC("Arduino Ready from ArduinoPC.ino");
  //Serial.print("sent ready message");
  delay(500);
  blinkLED(5); // just so we know it's alive
}

//================

void loop() {

  getSerialPort2Data();
  
  processData();

}

//================

void getSerialPort2Data() {

     // Receives data into tempBuffer[]
     //   saves the number of bytes that the PC said it sent - which will be in tempBuffer[1]
     //   uses decodeHighBytes() to copy data from tempBuffer to dataRecvd[]
     
     // the Arduino program will use the data it finds in dataRecvd[]

  if(SerialPort2.available() > 0) {

    byte x = SerialPort2.read();
    if (x == startMarker) { 
      bytesRecvd = 0; 
      inProgress = true;
      // blinkLED(2);
      // debugToPC("start received");
    }
      
    if(inProgress) {
      tempBuffer[bytesRecvd] = x;
      bytesRecvd ++;
    }

    if (x == endMarker) {
      inProgress = false;
      allReceived = true;
      //debugToPC("end received");

        // save the number of bytes that were sent
      dataSentNum = tempBuffer[1];
  
      decodeHighBytes();
    }
  }
}

//============================

void processData() {
    // processes the data that is in dataRecvd[]

  if (allReceived) {
  
      // for demonstration just print dataRecvd
    for (byte n = 0; n < dataRecvCount; n++) {


           Serial.print(n);
           Serial.print(":  ");
           Serial.print(int(char(dataRecvd[n])));
           Serial.println();
    
      }
//      else if(int(char(dataRecvd[n])) == 115){
//        //strafe channel
//        strafeWifiVal = int(char(dataRecvd[n+2]))
//      }    
//      else if(int(char(dataRecvd[n])) == 116){
//        //turn channel
//        turnWifiVal = int(char(dataRecvd[n+3]))
//      }

    
    
    
    //}

//    dataToPC();
    delay(100);
    allReceived = false; 
  }
}
int combine(int x, int y)
{
    int z;
    if(y >= 10)
        x *= 10;
    x *= 10;
    z = x + y;
    return z;
}
//============================

void decodeHighBytes() {

  //  copies to dataRecvd[] only the data bytes i.e. excluding the marker bytes and the count byte
  //  and converts any bytes of 253 etc into the intended numbers
  //  Note that bytesRecvd is the total of all the bytes including the markers
  dataRecvCount = 0;
  for (byte n = 2; n < bytesRecvd - 1 ; n++) { // 2 skips the start marker and the count byte, -1 omits the end marker
    byte x = tempBuffer[n];
    if (x == specialByte) {
       // debugToPC("FoundSpecialByte");
       n++;
       x = x + tempBuffer[n];
    }
    dataRecvd[dataRecvCount] = x;
    dataRecvCount ++;
  }
}

//=========================

void debugToPC( char arr[]) {
    byte nb = 0;
    SerialPort2.write(startMarker);
    SerialPort2.write(nb);
    SerialPort2.print(arr);
    SerialPort2.write(endMarker);
}

//=========================

void debugToPC( byte num) {
    byte nb = 0;
    SerialPort2.write(startMarker);
    SerialPort2.write(nb);
    SerialPort2.print(num);
    SerialPort2.write(endMarker);
}

//=========================

void blinkLED(byte numBlinks) {
    for (byte n = 0; n < numBlinks; n ++) {
      digitalWrite(13, HIGH);
      delay(200);
      digitalWrite(13, LOW);
      delay(200);
    }
}
