#include <ArduinoJson.h>
#include <SoftwareSerial.h> //this is needed for alternate serial pin designations on Teensy 3
#define TX3_PIN 8
#define RX3_PIN 7
SoftwareSerial  SerialPort2(RX3_PIN, TX3_PIN);

int driveval = 0;
int turnval = 0;
int strafeval = 0;

const char startMarker = '<';
const char endMarker = '>';

void setup() {
  // put your setup code here, to run once:
  pinMode(13, OUTPUT); // the onboard LED
  SerialPort2.begin(57600);
  Serial.begin(9600);
  delay(10000);

  debugToPC("Arduino Ready from ArduinoPC.ino");
  //Serial.print("sent ready message");
  delay(500);  //Serial.print("sent ready message");
}


unsigned long curMillis;
unsigned long prevCheckWifi = 0;
unsigned long checkWifiInterval = 50; //checks wifi for new data every 100 ms


void loop() {
  curMillis = millis();
  
//  if(((curMillis - prevCheckWifi) > checkWifiInterval)){
    readWifi();
//    Serial.println();
//    Serial.print("Updating Wifi");
//    Serial.println();
    prevCheckWifi = curMillis;
//  }
 
}

bool readWifi(void){
if (SerialPort2.available() > 2){
   StaticJsonBuffer<200> jsonBuffer;

  // put your main code here, to run repeatedly:
   JsonObject& root = jsonBuffer.parseObject(SerialPort2);

    // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return false;
  }
 driveval = root["drive"];
 turnval = root["turn"];
 strafeval = root["strafe"];

 Serial.print("driveVal: ");
 Serial.print(driveval);
 Serial.println();
 
 Serial.print("strafeVal: ");
 Serial.print(strafeval);
 Serial.println();
 
 Serial.print("turnVal: ");
 Serial.print(turnval);
 Serial.println();
// Serial.println();
return true;
}
//Serial.println("Serial not available");
}

void debugToPC( byte num) {
    byte nb = 0;
    SerialPort2.write(startMarker);
    SerialPort2.write(nb);
    SerialPort2.print(num);
    SerialPort2.write(endMarker);
}
void debugToPC( char arr[]) {
    byte nb = 0;
    SerialPort2.write(startMarker);
    SerialPort2.write(nb);
    SerialPort2.print(arr);
    SerialPort2.write(endMarker);
}

