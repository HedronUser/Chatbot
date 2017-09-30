#include <ArduinoJson.h>

void setup(){
    Serial.begin(9600);

}

void loop(){
    //
    // Step 1: Reserve memory space
    //
    StaticJsonBuffer<200> jsonBuffer;

    //
    // Step 2: Build object tree in memory
    //
    JsonObject& root = jsonBuffer.createObject();
    root["sensor"] = "gps";
    root["time"] = millis();

    JsonArray& data = root.createNestedArray("data");
    data.add(48.756080);
    data.add(2.302038);

    //
    // Step 3: Generate the JSON string
    //
    root.printTo(Serial);
    Serial.println();
    delay(1000);
}

