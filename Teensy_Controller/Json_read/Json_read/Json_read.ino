#include <ArduinoJson.h>

void setup() {
  // put your setup code here, to run once:
 
  
  // JSON input string.
  //
  // It's better to use a char[] as shown here.
  // If you use a const char* or a String, ArduinoJson will
  // have to make a copy of the input in the JsonBuffer.

  // Root of the object tree.
  //
  // It's a reference to the JsonObject, the actual bytes are inside the
  // JsonBuffer with all the other nodes of the object tree.
  // Memory is freed when jsonBuffer goes out of scope.
  Serial.begin(9600);

  }

//  char json[] = "{\"drive\":0,\"strafe\":0,\"turn\":0}";



void loop() {
    StaticJsonBuffer<200> jsonBuffer;

  // put your main code here, to run repeatedly:
   JsonObject& root = jsonBuffer.parseObject(Serial);
  // Test if parsing succeeds.
  if (!root.success()) {
    Serial.println("parseObject() failed");
    return;
  }
     int driveval = root["drive"];
     int turnval = root["turn"];
     int strafeval = root["strafe"];
//
 Serial.print("driveVal: ");
 Serial.print(driveval);
 Serial.println();
// Serial.println();

}
