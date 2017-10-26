// Written by Robin2 April 2014
//
// this sketch contains examples of different ways to read serial data in an Arduino.
// each version is in its own function, called from loop().
// only one of the function calls should be uncommented at any one time.

//================
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3
#define TX3_PIN 8
#define RX3_PIN 7
SoftwareSerial  SerialPort2(RX3_PIN, TX3_PIN);

char inputChar = 'X';
byte inputByte = 255;

const byte buffSize = 32;
char inputSeveral[buffSize]; // space for 31 chars and a terminator

byte maxChars = 12; // a shorter limit to make it easier to see what happens
                           //   if too many chars are entered

int inputInt = 0;
float inputFloat = 0.0;
char inputCsvString[12];


//===================

void setup() {
  Serial.begin(9600);
  Serial.println("Starting DemoDataInput.ino");
  SerialPort2.begin(115200);
}

//===================

void loop() {

  readCSV();
  
  delay(800);
}
//===================

void readCSV() {

      // this function expects a series of comma separated values
      // for this demo the sequence of items must be a string, an integer, a float
      // for example testing, 123 , 4567.89
      // spaces around the commas are optional
   
        // first read severalChars into the array inputSeveral

    inputSeveral[0] = 0;
    maxChars = buffSize - 1; // use full size of buffer for this function
    byte charCount = 0;  
    byte ndx = 0;        
    
    if (SerialPort2.available() > 0) {
      while (SerialPort2.available() > 0) { 
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        } 
        inputSeveral[ndx] = SerialPort2.read();
        ndx ++;        
        charCount ++;
      }
      if (ndx > maxChars) { 
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0; 
    }

      // now we need to split the received string into its parts
      // this is done by strtok() which looks for the token - the comma in this case

    char * partOfString; // this is used by strtok() as an index
    
    partOfString = strtok(inputSeveral,",");      // get the first part - the string
    strcpy(inputCsvString, partOfString); // copy it to inputCsvString
    
    partOfString = strtok(NULL, ","); // this continues where the previous call left off
    inputInt = atoi(partOfString);     // convert this part to an integer
    
    partOfString = strtok(NULL, ","); 
    inputFloat = atof(partOfString);     // convert this part to a float
    

    Serial.print("String -- ");
    Serial.print(inputCsvString);
    Serial.print("  Int -- ");
    Serial.print(inputInt);
    Serial.print("  Float -- ");
    Serial.println(inputFloat);

}

