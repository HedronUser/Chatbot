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
#define HIGH_ACCURACY
//
//ros::NodeHandle nh;
//std_msgs::UInt8 distance_msg;
//ros::Publisher distance_publisher("tof_range_finder", &distance_msg);

const size_t BUFFER_SIZE = JSON_ARRAY_SIZE(24) + JSON_OBJECT_SIZE(1);

//   MUX 1  sensors 1-16
//Mux control pins plug into pin 2,3,4,5 on teensy
int s0 = 6;
int s1 = 7;
int s2 = 8;
int s3 = 9;
int mux1en = 11; //ENABLE PIN

//  MUX 2 sensors 17-24
int d0 = 2;
int d1 = 3;
int d2 = 4;
int d3 = 5;
int mux2en = 10;  //ENABLE PIN

// Potentiometer to trim the sensitivity of array
int potval = 0;
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
  
  
  digitalWriteFast(s0, LOW);
  digitalWriteFast(s1, LOW);
  digitalWriteFast(s2, LOW);
  digitalWriteFast(s3, LOW);
  digitalWriteFast(mux1en, LOW); //initialize the mux 1 ON
  
  
  digitalWriteFast(d0, LOW);
  digitalWriteFast(d1, LOW);
  digitalWriteFast(d2, LOW);
  digitalWriteFast(d3, LOW);
  digitalWriteFast(mux2en, HIGH); //initialize the mux 2 OFF
  
  
  Serial.begin(115200);
  
  
    
  //  VL6180 and ROS  Publisher  
  // ros initialization:
  //  nh.initNode();
  //  nh.advertise(distance_publisher);
  
    // sensor initialization:
    Wire.begin(); //Start I2C library
    Wire.setClock(10000);
    delay(1000); // delay .1s
  
  //initialize MUX 1 - sensors 1-15 (channel 0 - 14) avoid last channel
  for(int i = 0; i < 15; i ++){ 
      initializeMux1(i); //assign the returned value to a number
      delay(1); 
      }
  
  digitalWriteFast(mux1en, HIGH); //turn mux 1 OFF
  digitalWriteFast(mux2en, LOW); //turn mux 2 ON

  //initialize sensor 16 and 24
  initializeMux2(0); //sensor 16 set at channel 0 
  delay(1);
  initializeMux2(1); //sensor 24 set at channel 1
  delay(1);
  //initialize MUX 2 - sensors 17-23
  for(int j = 8; j < 15; j ++){ 
      initializeMux2(j); //assign the returned value to a number
      delay(1);
    } 

  delay(1);
  digitalWriteFast(mux2en, HIGH); //turn mux 2 OFF
  digitalWriteFast(mux1en, LOW); //turn mux 1 ON
  //delay(100);
//  Serial.print("changeD");
}

int sensorArray[] = {1000,1000,600,1000,1000,1000, 1000,1000,600,1000,1000,1000, 1000,1000,600,1000,1000,1000, 1000,1000,600,1000,1000,1000, 0}; //fake test array
//int sensorArray[] = {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}; //holds 24 sensor values

int obstacledetection[] = {0, 0, 0, 0}; //initialize obstacle detection array to hold obstacle direction values (Front, Right, Rear, Left)  

//global to store data
int range = 0;

//
int controlPin1[] = {s0, s1, s2, s3};
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
   
int controlPin2[] = {d0, d1, d2, d3};


unsigned long curMillis = 0;
unsigned long prevMillis = 0;
unsigned long newMillis = 0;
unsigned long deltaMillis = 0;
unsigned long elapsedTime =  0;

unsigned long readInterval = 20;

void loop(){

    curMillis = millis();
    readmuxes();
    //parsearray(); //processes sensor data and writes to the obstacle detection array
//    potval = analogRead(0); //read the value of trimpot
//    Serial.print("analog 0 is: ");
//    Serial.println(potval);
    //delay(10);
    if ((curMillis - prevMillis) > readInterval){
          publishvalues();
          prevMillis = curMillis;
    }
      
} 

void readmuxes(void){
  // Read MUX 1
  for(int k = 0; k < 15; k ++){ 
    //reads into 1-14
    readMux1(k);
//    Serial.print("Value at channel "); 
//    Serial.print(k); 
//    Serial.print("is : "); 
//    Serial.print(sensorArray[k]); 
    delay(1);
   }
  
  //Serial.print(sensorArray);
  //delay(10);
  digitalWriteFast(mux1en, HIGH); //turn mux 1 OFF
  digitalWriteFast(mux2en, LOW); //turn mux 2 ON
  delay(1);
  //read sensor 15 twice
  readMux2(0);
  readMux2(0);
//  Serial.print("Value at channel 15is : ");
//  Serial.print(sensorArray[15]);
  // Read MUX 2
  delay(1);
    //read sensor 23 twice
  readMux2(1);
  readMux2(1);

  delay(1);
  readMux2(2);
  delay(1);
  for(int l = 8; l < 15; l ++){ 
    //reads into 16-23
    readMux2(l);
    delay(1);
//    Serial.print("Value at channel "); 
//    Serial.print(l + 8); 
//    Serial.print("is : "); 
//    Serial.print(sensorArray[l+8]); 
   }

//  Serial.print("Value at channel 23is : ");
//  Serial.print(sensorArray[23]);
  //delay(10);
  digitalWriteFast(mux2en, HIGH); //turn mux 2 OFF
  digitalWriteFast(mux1en, LOW); //turn mux 1 ON
  //delay(10);
}
  
void publishvalues(void){
//this should publish the values over serial that are processed from parsing function as four numbers (0, 0, 0, 0)
//indexed as (Front, Left, Rear, Right) with 0 indicating no object above threshold (potval) and 1 indicating an object
//is present that is below threshold value
 
 // Serial.print("Incoming Obstacle Data");
    
    StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonArray& channel_data = root.createNestedArray("channel_data");
    for( int i = 0; i < 24; i++){   //for 24 channels
     channel_data.add(sensorArray[i]);   //REAL DATA <-- Jesse!
     //channel_data.add(1);            //FAKE DATA
    }
    root.printTo(Serial);
    Serial.println();           //ln return is required for successful parsing

    //delay(1000); //Why a delay of 1 second?

 
//  for (int i = 0; i < 4; i++){
//    if (i == 0){
//      Serial.println();
//      Serial.print("front");
//      Serial.println();
//      }
//    if (i == 1){
//      Serial.print("left");
//      Serial.println();
//      }
//    if (i == 2){
//      Serial.print("rear");
//      Serial.println();
//      }
//    if (i == 3){
//      Serial.print("right");
//      Serial.println();
//      }
//    Serial.print(obstacledetection[i]);
//    Serial.println();
//    //delay(100);
//  }
}

void parsearray(void){
//this should take the measurements from sensors designated as sensors 1-6 as front obstacle, sensors 7-12 are right obstacle, 
//sensors 13-18 as rear obstacle and sensors 19-24 as left obstacle and compare them to threshold value from potentiometer
// if they are less than the value then change variables front, rear, left, and right respectively to 1.
  
   //initialize array to 0 to reassign obstacle direction values
  for(int i = 0; i < 4; i++){ 
    obstacledetection[i] = 0; 
   }
        
  //for front sensors
  for (int j = 0; j < 6; j++){     //read the first six values into sensorvalue array and check if they are less than pot setting
                              // if so then set first index of obstacledetection array to 1
     if (sensorArray[j] < potval){
      obstacledetection[0] = 1;
     }
  }
  
    //for right sensors
  for (int k = 6; k < 12; k++){
     if (sensorArray[k] < potval){
      obstacledetection[1] = 1;
     }
  }
  
    //for rear sensors
  for (int l = 12; l < 18; l++){
     if (sensorArray[l] < potval){
      obstacledetection[2] = 1;
     }
  }
   
    //for left sensors
  for (int m = 18; m < 24; m++){
     if (sensorArray[m] < potval){
      obstacledetection[3] = 1;
     }
  }

    //print the obstacle detection array to confirm it works
//   for(int n = 0; n < 4; n++){
//    Serial.print(obstacledetection[n]);
//    Serial.print(" "); 
//   }
}


void readMux1(int chan){ //points to global sensorarray
    //returns the range of the sensor specified using the channel variable as input for muxer

    for(int i = 0; i < 4; i ++){
      digitalWriteFast(controlPin1[i], muxChannel[chan][i]);
      } 

    
    range = sensor.readRangeContinuousMillimeters(); //bind distance of sensor to range variable
    
    //check to see if value is > 8190 and reinitialize
    if (range > 8200){
      //initializeMux1(chan);  
    }

    if (range == 0){
      range = 8190;
    }
      
      //write into array
    sensorArray[chan] = range;

   //Serial.print(sensor.readRangeSingleMillimeters());
   if (sensor.timeoutOccurred()) { 
//    Serial.print(" TIMEOUT"); 

    //initializeMux1(chan);
   }
   //Serial.println();

//    distance_msg.data = range; //publish as ros msg 
//
////    distance_publisher.publish( &distance_msg);
//
//    nh.spinOnce();
//    
}

void readMux2(int chan){
    //returns the range of the sensor specified using the channel variable as input for muxer

    for(int i = 0; i < 4; i ++){
      digitalWriteFast(controlPin2[i], muxChannel[chan][i]);
      } 

    
   //Serial.print(sensor.readRangeSingleMillimeters());
   if (sensor.timeoutOccurred()) { 
//    Serial.print(" TIMEOUT");
      //initializeMux2(chan); 
    }
   //Serial.println();
    if (chan == 2){
      //do nothing
    }
    else{
   //broke here
   range = sensor.readRangeContinuousMillimeters(); //bind distance of sensor to range variable
     }
      //check to see if value is > 8190 and reinitialize
    if (range > 8200){
      //initializeMux2(chan);  
    }
  
  //bind channel 15 to right place in array
  if (chan == 0){
    sensorArray[15] = range;
    
  }
  else if (chan == 1){
    sensorArray[23] = range;
  }
  else if (chan == 2){
    //do nothing
  }
  else{  
   sensorArray[chan + 8] = range; //write into 2nd part of array
  }
   //Serial.print(range);

    
//    distance_msg.data = range; //publish as ros msg 
//
////    distance_publisher.publish( &distance_msg);
//
//    nh.spinOnce();

}
    

void initializeMux1(int chan){
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
      digitalWriteFast(controlPin[i], muxChannel[chan][i]);
      }; 
    
    sensor.init();
    sensor.setTimeout(0); //STOCK IS 500

    
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
  sensor.setMeasurementTimingBudget(33000);
#endif
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  
  sensor.startContinuous(); //continuous mode
    
}

void initializeMux2(int chan){
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
      digitalWriteFast(controlPin[i], muxChannel[chan][i]);
      } 

    sensor.init();

    sensor.setTimeout(0); //50 initial , 150 

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
  sensor.setMeasurementTimingBudget(33000);
#endif
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  
  sensor.startContinuous(); //continuous mode
    
}



