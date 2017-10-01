//   VL53L0X and ROS  Publisher
//
//#include <ros.h>
//#include <std_msgs/UInt8.h>

#include <Wire.h>
#include <VL53L0X.h>
#include <ArduinoJson.h>

VL53L0X sensor;



////DO WE NEED THIS DOCUMENTATION?///////////////
//
// Uncomment this line to use long range mode. This
// increases the sensitivity of the sensor and extends its
// potential range, but increases the likelihood of getting
// an inaccurate reading because of reflections from objects
// other than the intended target. It works best in dark
// conditions.

//#define LONG_RANGE

// Uncomment ONE of these two lines to get
// - higher speed at the cost of lower accuracy OR
// - higher accuracy at the cost of lower speed

//#define HIGH_SPEED
//#define HIGH_ACCURACY
//
//ros::NodeHandle nh;
//std_msgs::UInt8 distance_msg;
//ros::Publisher distance_publisher("tof_range_finder", &distance_msg);
//
////END DO WE NEED THIS DOCUMENTATION?////////////



////GLOBAL VARIABLES/////////////////////////////
//

// Potentiometer to trim the sensitivity of array
int potval = 0;


//   MUX 1  sensors 1-16
//Mux control pins plug into pin 2,3,4,5 on teensy
int s0 = 2;
int s1 = 3;
int s2 = 4;
int s3 = 5;
int mux1en = 6; //ENABLE PIN
int mux_1_ControlPin[] = {s0, s1, s2, s3}; //Must be Int[4]

//  MUX 2 sensors 17-24
int d0 = 7;
int d1 = 8;
int d2 = 9;
int d3 = 10;
int mux2en = 11;  //ENABLE PIN
int mux_2_ControlPin[] = {d0, d1, d2, d3}; //Must be Int[4]


//Look up array used in conjunction with $_ControlPin[] to activate
//The correct digitalpins for a given channel
int muxChannel[16][4]={ {0,0,0,0}, //channel 0
{1,0,0,0}, //channel 1
{0,1,0,0}, //channel 2
{1,1,0,0}, //channel 3
{0,0,1,0}, //channel 4
{1,0,1,0}, //channel 5
{0,1,1,0}, //channel 6
{1,1,1,0}, //channel 7
{0,0,0,1}, //channel 8
{1,0,0,1}, //channel 9
{0,1,0,1}, //channel 10
{1,1,0,1}, //channel 11
{0,0,1,1}, //channel 12
{1,0,1,1}, //channel 13
{0,1,1,1}, //channel 14
{1,1,1,1} //channel 15
};
//
//////////////////////////////////////////////////



////START SETUP////////////////////////////////////////
//
void setup(){
    //    Initialize MUX 1 and MUX 2
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    pinMode(mux1en, OUTPUT);

    pinMode(d0, OUTPUT);
    pinMode(d1, OUTPUT);
    pinMode(d2, OUTPUT);
    pinMode(d3, OUTPUT);
    pinMode(mux2en, OUTPUT);

    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);
    digitalWrite(mux1en, LOW); //initialize the mux 1 ON

    digitalWrite(d0, LOW);
    digitalWrite(d1, LOW);
    digitalWrite(d2, LOW);
    digitalWrite(d3, LOW);
    digitalWrite(s3, LOW);
    digitalWrite(mux2en, HIGH); //initialize the mux 2 OFF


    ////BEGIN SERIAL///////////////////
    //
    //int baudrate = 115200;
    int baudrate = 9600;
    Serial.begin(baudrate);
    ////////////


    //// SENSOR INITIALIZATION //////////////////
    //
    Wire.begin(); //Start I2C library
    delay(1000); // delay .1s

    //initialize MUX 1 - sensors 1-16
    for(int i = 0; i < 16; i ++){
        initializeMux(i, mux_1_ControlPin); //TODO?: assign the returned value to a number
    }
    digitalWrite(mux1en, HIGH); //turn mux 1 OFF
    digitalWrite(mux2en, LOW); //turn mux 2 ON


    //initialize MUX 2 - sensors 17-24
    for(int j = 0; j < 8; j ++){
        initializeMux(j, mux_2_ControlPin); //TODO?: assign the returned value to a number
    }
    digitalWrite(mux1en, LOW); //turn mux 1 ON
    digitalWrite(mux2en, HIGH); //turn mux 2 OFF
    //
    //////////////////////////////////////////////
}
//
////END SETUP//////////////////////////////////////////



////NOTE: What is this sensorArray?
// int sensorArray[] = {1000,1000,600,1000,1000,1000, 1000,1000,600,1000,1000,1000, 1000,1000,600,1000,1000,1000, 1000,1000,600,1000,1000,1000}; //fake test array
// int sensorArray[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; //holds 24 sensor values
int sensorArray[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; //holds 16 sensor values



void loop(){
    ////PRINT SERIAL JSON CHANNEL DATA///////////////
    //
    // JSON output:
    // "channel_data": array[24], each array has readMux output
    // "potval": analogRead of pin 0
    //
    //Potential TODO: rename the array to be individual locations for better
    //portability
    //
    //Consult ArduinoJson for additional examples and data types.
    StaticJsonBuffer<200> jsonBuffer; //TODO: might be a dynamic buffer
    JsonObject& root = jsonBuffer.createObject();

    root["potval"] = analogRead(0);  //Read trimpot value

    JsonArray& channel_data = root.createNestedArray("channel_data");
    for( int i = 0; i < 15; i++){   //for 16 channels
        channel_data.add(readMux(i, mux_1_ControlPin));
    }
    for( int j = 17; j < 25; j++){   //for remaining 8 channels
        channel_data.add(readMux(j, mux_2_ControlPin));
    }

    root.printTo(Serial);
    Serial.println();//parser looks for println carriage return \r\n

    delay(1000); //Why a delay of 1 second?
    //
    /////PRINT SERIAL JSON CHANNEL DATA//////////////
}
/////END LOOP()//////////////////////////////////////



//TODO: move this method into the main loop for the most part
void serialTimeoutJSON(){
    StaticJsonBuffer<20> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["error"] = "sensor_timeout";
    root.printTo(Serial);
    Serial.println();
}


int readMux(int channel, int controlPin[]){
    //returns the range of the sensor specified using the channel variable as input for muxer
    for(int i = 0; i < 4; i ++){
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }

    if (sensor.timeoutOccurred()) {
        serialTimeoutJSON();
    }

    int range = sensor.readRangeSingleMillimeters(); //bind distance of sensor to range variable

    return range;
}



void initializeMux(int chan, int controlPin[]){
    // pass this function a channel and integer value to write to the I2C port
    // it will return a new value that is returned

    //loop through the 4 sig to set the pins for mux channel
    for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[chan][i]);
    }

    sensor.init();
    sensor.setTimeout(50);

    #if defined LONG_RANGE
        // lower the return signal rate limit (default is 0.25 MCPS)
        sensor.setSignalRateLimit(0.1);
        // increase laser pulse periods (defaults are 14 and 10 PCLKs)
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 18);
        sensor.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 14);
    #endif

    #if defined HIGH_SPEED
        // reduce timing budget to 20 ms (default is about 33 ms)
        sensor.setMeasurementTimingBudget(20000);
    #elif defined HIGH_ACCURACY
        // increase timing budget to 200 ms
        sensor.setMeasurementTimingBudget(200000);
    #endif
    // Start continuous back-to-back mode (take readings as
    // fast as possible).  To use continuous timed mode
    // instead, provide a desired inter-measurement period in
    // ms (e.g. sensor.startContinuous(100)).

    sensor.startContinuous(); //continuous mode
} //END initializeMux
