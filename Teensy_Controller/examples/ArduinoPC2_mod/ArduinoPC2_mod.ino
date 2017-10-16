

const byte buffSize = 40;
char inputBuffer[buffSize];
const char startMarker = '<';
const char endMarker = '>';
byte bytesRecvd = 0;
boolean readInProgress = false;
boolean newDataFromPC = false;
unsigned long prevLEDmillis = 0;


char messageFromPC[buffSize] = {0};
int newFlashInterval = 0;
int LEDinterval = 0;
unsigned long curMillis;

unsigned long prevReplyToPCmillis = 0;
unsigned long replyToPCinterval = 1000;


float servoFraction = 0;
//=============

void setup() {
  Serial.begin(9600);
  
    // flash LEDs so we know we are alive

  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH); //on board LED
  
  delay(500); // delay() is OK in setup as it only happens once
  
  digitalWrite(13, LOW);
  
    // tell the PC we are ready
  Serial.println("<Arduino Ready>");
}

//=============

void loop() {
  curMillis = millis();
  getDataFromPC();
  updateFlashInterval();
  replyToPC();
  flashLEDs();
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

  if (newDataFromPC) {
    newDataFromPC = false;
    Serial.print("<Msg ");
    Serial.print(messageFromPC);
    Serial.print(" NewFlash ");
    Serial.print(newFlashInterval);
    Serial.print(" SrvFrac ");
    Serial.print(servoFraction);
    Serial.print(" Time ");
    Serial.print(curMillis >> 9); // divide by 512 is approx = half-seconds
    Serial.println(">");
  }
}

//============

void updateFlashInterval() {

   // this illustrates using different inputs to call different functions
  if (strcmp(messageFromPC, "LED1") == 0) {
     updateLED1();
  }
 
}

//=============

void updateLED1() {

  if (newFlashInterval > 100) {
    LEDinterval = newFlashInterval;
  }
}

//=============


//=============

void flashLEDs() {

 
    if (curMillis - prevLEDmillis >= LEDinterval) {
       prevLEDmillis += LEDinterval;
       digitalWrite( 13, ! digitalRead(13) );
    }
}

//=============


