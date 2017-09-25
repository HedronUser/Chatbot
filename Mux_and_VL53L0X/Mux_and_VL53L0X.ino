//   VL53L0X and ROS  Publisher
//
//#include <ros.h>
//#include <std_msgs/UInt8.h>

#include <Wire.h>

#include <VL53L0X.h>

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


//   MUX 1  sensors 1-16
//Mux control pins plug into pin 2,3,4,5 on teensy
int s0 = 2;
int s1 = 3;
int s2 = 4;
int s3 = 5;
int mux1en = 6; //ENABLE PIN

//  MUX 2 sensors 17-24
int d0 = 7;
int d1 = 8;
int d2 = 9;
int d3 = 10;
int mux2en = 11;  //ENABLE PIN

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


Serial.begin(115200);


  
//  VL6180 and ROS  Publisher  
// ros initialization:
//  nh.initNode();
//  nh.advertise(distance_publisher);

  // sensor initialization:
  Wire.begin(); //Start I2C library
  delay(1000); // delay .1s

//initialize MUX 1 - sensors 1-16
for(int i = 0; i < 16; i ++){ 
    initializeMux1(i); //assign the returned value to a number
  } 

digitalWrite(mux1en, HIGH); //turn mux 1 OFF
digitalWrite(mux2en, LOW); //turn mux 2 ON


//initialize MUX 2 - sensors 17-24
for(int i = 0; i < 8; i ++){ 
    initializeMux2(i); //assign the returned value to a number
  } 


digitalWrite(mux1en, LOW); //turn mux 1 ON
digitalWrite(mux2en, HIGH); //turn mux 2 OFF
}

int sensorArray[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; //holds 24 sensor values

void loop(){
// Read MUX 1

for(int i = 0; i < 16; i ++){ 
  readMux1(i);
  Serial.print("Value at channel "); 
  Serial.print(i); 
  Serial.print("is : "); 
  Serial.println(sensorArray[i]); 
 }

Serial.println(sensorArray);

digitalWrite(mux1en, HIGH); //turn mux 1 OFF
digitalWrite(mux2en, LOW); //turn mux 2 ON

// Read MUX 2
for(int i = 0; i < 8; i ++){ 
  readMux2(i);
  Serial.print("Value at channel "); 
  Serial.print(i); 
  Serial.print("is : "); 
  Serial.println(sensorArray[i]); 
 }
Serial.println(sensorArray);

digitalWrite(mux1en, LOW); //turn mux 1 ON
digitalWrite(mux2en, HIGH); //turn mux 2 OFF
  
} 
  
void readMux1(int chan){ //points to global sensorarray
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
      digitalWrite(controlPin[i], muxChannel[chan][i]);
      } 
    
   //Serial.print(sensor.readRangeSingleMillimeters());
   if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
   Serial.println();

    
    int range = sensor.readRangeSingleMillimeters(); //bind distance of sensor to range variable

    Serial.print(range);

    sensorArray[chan] = range; //write into 1st part of array

//    distance_msg.data = range; //publish as ros msg 
//
////    distance_publisher.publish( &distance_msg);
//
//    nh.spinOnce();
//    
}

void readMux2(int chan){
    //returns the range of the sensor specified using the channel variable as input for muxer
    int controlPin[] = {d0, d1, d2, d3};
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
      digitalWrite(controlPin[i], muxChannel[chan][i]);
      } 
    
   //Serial.print(sensor.readRangeSingleMillimeters());
   if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
   
   Serial.println();

    
   int range = sensor.readRangeSingleMillimeters(); //bind distance of sensor to range variable

   sensorArray[chan + 16] = range; //write into 2nd part of array
   
   Serial.print(range);

    
//    distance_msg.data = range; //publish as ros msg 
//
////    distance_publisher.publish( &distance_msg);
//
//    nh.spinOnce();
//    

}
    

void initializeMux1(int channel){
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

void initializeMux2(int channel){
    // pass this function a channel and integer value to write to the I2C port
    // it will return a new value that is returned
    int controlPin[] = {d0, d1, d2, d3};
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


