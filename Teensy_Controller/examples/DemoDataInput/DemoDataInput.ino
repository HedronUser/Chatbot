// Written by Robin2 April 2014
//
// this sketch contains examples of different ways to read serial data in an Arduino.
// each version is in its own function, called from loop().
// only one of the function calls should be uncommented at any one time.

//===================
// global variables used in the various functions

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
}

//===================

void loop() {

    // only one of these functions should be uncommented at any one time
    
  readSingleChar();
//  readSingleDigit();
//  readSeveralChars();
//  readOneFloat();
//  readCSV();
  
  delay(800);
}

//===================

void readSingleChar() {
     // this function reads a single character from the Serial Monitor
     // it assumes you type (e.g.) 'a' and press return
     // if there is nothing in the input buffer inputChar will be 'X'
     //   change this if you need to send an 'X'

     // watch what happens if you type 2 characters (e.g. ab) and then press return
     //   it is not designed for that

     // watch what happens if you type a digit and compare it with what happens in the 
     //     readSingleDigit function

  inputChar = 'X';
  if (Serial.available() > 0) {
    inputChar = Serial.read();
  }
  
    // a single charater can also be treated as a byte (for example A has the value 65)
    //   which means that it could represent any value from 0 - 255
    //   Of course it is difficult to send the non-printing characters with the Serial Monitor
    //     but they could be sent from a program on the PC
    
  inputByte = (byte) inputChar; // the "(byte)" tells the compiler to convert the char to a byte
  
    // the printing is just to show what happens
    // it would not be needed in 'real' programs
  Serial.print("InputChar --- ");
  Serial.print(inputChar);
  Serial.print("  InputByte --- ");
  Serial.println(inputByte);
}

//===================

void readSingleDigit() {

     // this function expets a single digit (0-9) from the Serial Monitor
     // it assumes you type (e.g.) '7' and press return
     // if there is nothing in the input buffer inputByte will be 255
  inputByte = 255;
  if (Serial.available() > 0) {
    inputByte = Serial.read();
    inputByte = inputByte - '0'; // subtracts the Ascii value for '0' (48) to give the numeric value
    if (inputByte < 0 | inputByte > 9) { // only values 0-9 are accepted
      inputByte = 255;
    }
  }
  
  Serial.print("InputValue --- ");
  Serial.println(inputByte);
}

//===================

void readSeveralChars() {

      // this reads all the characters in the input buffer
      // if there are too many for the inputSeveral array the extra chars will be lost

    inputSeveral[0] = 0; // makes inputSeveral an empty string with just a terminator
    
    byte charCount = 0;  // the number of characters actually received - some may be lost
    byte ndx = 0;        // the index position for storing the character
    
    if (Serial.available() > 0) {
    
      while (Serial.available() > 0) { // keep going until buffer is empty
        if (ndx > maxChars - 1) { // -1 because arrays count from 0
          ndx = maxChars;     // if there are too many chars the extra ones are 
        }                     //   dumped into the last array element which will
                              //   be overwritten by the string terminator
        inputSeveral[ndx] = Serial.read();
        ndx ++;        
        charCount ++;
      }
      
      if (ndx > maxChars) {  // to make sure the terminator is not written beyond the array
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0; // add a zero terminator to mark the end of the string
    }
    
    Serial.print("Num Chars Rcvd --- ");
    Serial.print(charCount);
    Serial.print("  ---  ");
    Serial.println(inputSeveral);
    
}

//===================

void readOneFloat() {

     // this function takes the characters from the Serial Monitor and converts them
     //   to a single floating point value using the function "atof()"
     
     // a similar approach can be used to read an integer value if "atoi()" is used

     // first read severalChars into the array inputSeveral
    inputSeveral[0] = 0; 
    byte charCount = 0;  
    byte ndx = 0;        
    
    if (Serial.available() > 0) {
      while (Serial.available() > 0) { 
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        } 
        inputSeveral[ndx] = Serial.read();
        ndx ++;        
        charCount ++;
      }
      if (ndx > maxChars) { 
        ndx = maxChars;
      }
      inputSeveral[ndx] = 0; 
    }

     // and then convert the string into a floating point number
     
    inputFloat = atof(inputSeveral); // atof gives 0.0 if the characters are not a valid number
     
    Serial.print("InputFloat --- ");
    Serial.println(inputFloat, 4); // the number specifies how many decimal places
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
    
    if (Serial.available() > 0) {
      while (Serial.available() > 0) { 
        if (ndx > maxChars - 1) {
          ndx = maxChars;
        } 
        inputSeveral[ndx] = Serial.read();
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

//===================
