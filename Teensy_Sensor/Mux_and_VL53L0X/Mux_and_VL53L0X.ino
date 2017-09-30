//   VL53L0X and ROS  Publisher
//
//#include <ros.h>
//#include <std_msgs/UInt8.h>

#include <Wire.h>

#include <VL53L0X.h>

#include <ArduinoJson.h>


VL53L0X sensor;

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


//   MUX
//Mux control pins plug into pin 2,3,4,5 on teensy
int s0 = 2;
int s1 = 3;
int s2 = 4;
int s3 = 5;

//Mux in “SIG” pin
//int SIG_pin = 18; //SDA pin on Teensyboard 18

void setup(){
    //Initialize MUX
    pinMode(s0, OUTPUT);
    pinMode(s1, OUTPUT);
    pinMode(s2, OUTPUT);
    pinMode(s3, OUTPUT);
    //pinMode(SIG_pin, INPUT);

    digitalWrite(s0, LOW);
    digitalWrite(s1, LOW);
    digitalWrite(s2, LOW);
    digitalWrite(s3, LOW);

    //int baudrate = 115200;
    int baudrate = 9600;
    Serial.begin(baudrate);
    //  VL6180 and ROS  Publisher
    // ros initialization:
    //  nh.initNode();
    //  nh.advertise(distance_publisher);

    // sensor initialization:
    Wire.begin(); //Start I2C library
    delay(1000); // delay .1s

    //initialize all muxed sensors by looping through them
    for(int i = 0; i < 16; i ++){
        initializeSensor(i); //assign the returned value to a number
    }
}

int sensorArray[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; //holds 16 sensor values

void loop(){
    ////PRINT SERIAL JSON CHANNEL DATA///////////////
    //
    //Creates an index array[16] with each index having  output from readMux
    //
    //Potential TODO: rename the array to be individual locations for better
    //portability
    //
    //Consult ArduinoJson for additional examples and data types.
    StaticJsonBuffer<200> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonArray& channel_data = root.createNestedArray("channel_data");
    for( int i = 0; i < 15; i++){   //for 16 channels
    // channel_data.add(readMux(i));   //REAL DATA <-- Jesse!
        channel_data.add(1);            //FAKE DATA
    }
    root.printTo(Serial);
    Serial.println();           //ln return is required for successful parsing

    delay(1000); //Why a delay of 1 second?
    //
    /////////////////////////////////////////////////
} //end void loop()

void serialTimeoutJSON(){
    StaticJsonBuffer<20> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    root["error"] = "sensor_timeout";
    root.printTo(Serial);
    Serial.println();
}


int readMux(int channel){
    //returns the range of the sensor specified using the channel variable as input for muxer
    int controlPin[] = {s0, s1, s2, s3};
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
    }; //loop through the 4 sig
    for(int i = 0; i < 4; i ++){
        digitalWrite(controlPin[i], muxChannel[channel][i]);
    }

    //Serial.print(sensor.readRangeSingleMillimeters());
    if (sensor.timeoutOccurred()) {
        serialTimeoutJSON();
    }

    int range = sensor.readRangeSingleMillimeters(); //bind distance of sensor to range variable
    //    distance_msg.data = range; //publish as ros msg
    //
    ////    distance_publisher.publish( &distance_msg);
    //
    //    nh.spinOnce();
    //
    //Serial.print(range);
    return range;
}

void initializeSensor(int channel){
    // pass this function a channel and integer value to write to the I2C port
    // it will return a new value that is returned
    int controlPin[] = {s0, s1, s2, s3};
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
    //loop through the 4 sig to set the pins for mux channel
    for(int i = 0; i < 4; i ++){
        digitalWrite(controlPin[i], muxChannel[channel][i]);
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
}
