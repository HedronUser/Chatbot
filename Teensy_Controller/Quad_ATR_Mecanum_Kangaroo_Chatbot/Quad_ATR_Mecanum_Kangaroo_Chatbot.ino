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
*/

// ****************************************************
// Libraries
// ****************************************************
#include <Kangaroo.h>
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3

// ****************************************************
// Motor Controller Initialization
// ****************************************************

// Teensy TX (pin 11) goes to Kangaroo S1
// Teensy RX (pin 10) goes to Kangaroo S2
// Teensy GND         goes to Kangaroo 0V
// Teensy 5V input    goes to Kangaroo 5V (OPTIONAL, if you want Kangaroo to power the Teensy)
#define TX_PIN 10
#define RX_PIN 9

// Assign the serial COM to the Kangaroo as Serial1 on the Teensy
SoftwareSerial  SerialPort(RX_PIN, TX_PIN);
KangarooSerial  K(SerialPort);

// Initialize our Kangroo objects. The named channels (1, 2, 3, or 4) must be configured on
// the Kangroo itself. This will require using the Describe software as well as a USB to TTL
// cable. 
// http://www.dimensionengineering.com/info/describe
KangarooChannel KR1(K, '3', 128);
KangarooChannel KR2(K, '4', 128);

KangarooChannel KF1(K, '1', 128);
KangarooChannel KF2(K, '2', 128);


// ****************************************************
// Initial setup function, called once
// RETURNS: none
// ****************************************************
void setup() {
  
  SerialPort.begin(115200);   // Initialize our Serial to 115200. This seems to be
  //Serial.listen();      // the most reliable baud rate to the kangaroo.
  
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

//K1.serialTimeout(1000); // If we don't send anything to the Kangaroo for 1 second (1000 ms),
                          // it will abort and hold position (if the last command was position)
                          // or hold velocity (if the last command was velocity).
                          // Since serial timeout is intended as a form of safety abort, to
                          // recover from a serial timeout, start() has to be issued again.
                          // Try disconnecting the TX line from S1 temporarily to see it in action.

}

// ****************************************************
// Main program loop. We'll cycle through commands here
// RETURNS: none
// ****************************************************
void loop() {
  
  long scalar = 0;               // Temporary values to hold
  long speedTemp1, speedTemp2;   // our assigned speeds

  // Demonstrates forward movement
  for (scalar = 0; scalar < 400000; scalar++) {
    
    speedTemp1 = scalar;
    speedTemp2 = 0 - scalar;
    
    KF1.s(speedTemp1);
    KF2.s(speedTemp1);    
    KR1.s(speedTemp2);
    KR2.s(speedTemp2);

    // can use KF1.s(speed).wait(); instead if desired
    delay(100);
    scalar = scalar + 20000;
  }
  
  // Demonstrates backwards movement
  for (scalar = 0; scalar < 400000;  scalar++) {
    
    speedTemp1 = 0 - scalar;
    speedTemp2 = scalar;
    
    KF1.s(speedTemp1);
    KF2.s(speedTemp1);    
    KR1.s(speedTemp2);
    KR2.s(speedTemp2);

    delay(100);
    scalar = scalar + 20000;
  }  

  // Demonstrates rotate right movement
  for (scalar = 0; scalar < 400000;  scalar++) {
    
    speedTemp1 = scalar;
    speedTemp2 = 0 - scalar;
    
    KR1.s(speedTemp1);
    KF1.s(speedTemp1);
    KF2.s(speedTemp2);
    KR2.s(speedTemp2);

    delay(100);
    scalar = scalar + 20000;
  }
  
  // Demonstrates rotate left movement
  for (scalar = 0; scalar < 400000;  scalar++) {
    
    speedTemp1 = 0 - scalar;
    speedTemp2 = scalar;
    
    KR1.s(speedTemp1);
    KF1.s(speedTemp1);
    KF2.s(speedTemp2);
    KR2.s(speedTemp2);

    delay(100);
    scalar = scalar + 20000;
  } 
  
    // Demonstrates shift left movement
  for (scalar = 0; scalar < 400000;  scalar++) {
    
    speedTemp1 = scalar;
    speedTemp2 = 0 - scalar;
    
    KR1.s(speedTemp1);
    KF2.s(speedTemp1);    
    KF1.s(speedTemp2);
    KR2.s(speedTemp2);

    delay(100);
    scalar = scalar + 20000;
  }
  
  // Demonstrates shift right movement
  for (scalar = 0; scalar < 400000;  scalar++) {
    
    speedTemp1 = 0 - scalar;
    speedTemp2 = scalar;
    
    KR1.s(speedTemp1);
    KF2.s(speedTemp1);    
    KF1.s(speedTemp2);
    KR2.s(speedTemp2);

    delay(100);
    scalar = scalar + 20000;
  } 


}

void powerOff(void){
    //this function can be used to save power when bot is not being driven
    // Notice that the Kangaroo provides no special resistance while the channel is powered down.
 // KF1.powerDown();
 // KF2.powerDown();
 // KR1.powerDown();
 // KR2.powerDown();
  }
