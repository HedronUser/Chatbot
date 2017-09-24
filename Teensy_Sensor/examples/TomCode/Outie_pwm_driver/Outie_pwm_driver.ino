/*
 * LOTs Arduino PWM shield
 * Raspberry PI sends pin ID and PWM rate over serial data via USB cable
 * parsed into individual signals and drives mosfets with a PWM signal using the sparkfun mosfet shield
 * https://www.sparkfun.com/products/10618
 *
 * 1:30 is pin 1, PWM 30
 * also accepts two or more values comma seperated up to 38 bytes
 * 1:30,5:254,9:75
 * terminated wiht a carraige return '/n'
 *
 * Arduino PWM pins are 3,5,6,9,10,11
 *
 * Disable Auto Reset on Serial connection:
 * Stick a 120 ohm resistor in the headers between 5v and reset on the arduino

 */

// pins for the PWM circuits
const int pwmPin3 = 3;
const int pwmPin5 = 5;
const int pwmPin6 = 6;
const int pwmPin9 = 9;
const int pwmPin10 = 10;
const int pwmPin11 = 11;

const int sizeOfInData = 40;
char inData[sizeOfInData];
byte indexer = 0;

char *pwm1, *pwm2, *pwm3, *pwm4, *pwm5, *pwm6, *i;

void setup() {
  Serial.begin(115200);
  Serial.setTimeout(100);
  // make the pins outputs:
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin5, OUTPUT);
  pinMode(pwmPin6, OUTPUT);
  pinMode(pwmPin9, OUTPUT);
  pinMode(pwmPin10, OUTPUT);
  pinMode(pwmPin11, OUTPUT);
}
void loop() {
  while(Serial.available() > 0){
    char aChar = Serial.read();
      if(aChar == '\n')  //this is the return character.... so all data is received from the PI... do stuff with the data
      {
        for(int i = 0; i < indexer; i++){
          Serial.write(inData[i]); //writes back to the pi just so you can see it there
        }
        Serial.println();
          char* command = strtok(inData, ",");
          while (command != 0)
          {
            // Split the command in two values
            char* separator = strchr(command, ':');
            if (separator != 0){
              // Actually split the string in 2: replace ':' with 0
              *separator = 0;
              int pinId = atoi(command);
              ++separator;
              int pwmRate = atoi(separator);

              switch (pinId){
                case 3:    //
                  analogWrite(pwmPin3, pwmRate);
                  break;
                case 5:    //
                  analogWrite(pwmPin5, pwmRate);
                  break;
                case 6:    //
                  analogWrite(pwmPin6, pwmRate);
                  break;
                case 9:    //
                  analogWrite(pwmPin9, pwmRate);
                  break;
                case 10:    //
                  analogWrite(pwmPin10, pwmRate);
                  break;
                case 11:    //
                  analogWrite(pwmPin11, pwmRate);
                  break;
              }
              Serial.print("pinId: ");
              Serial.println(pinId);
              Serial.print("pwmRate: ");
              Serial.println(pwmRate);
            }
            // Find the next command in input string
            command = strtok(0, "&");
          }
        indexer = 0;
        inData[indexer] = NULL; //empties the array
      }else{
        if(indexer < sizeOfInData-2){  //safety to keep from overrunning the buffer
          inData[indexer] = aChar;
          indexer++;
        inData[indexer] = '\0'; // Keep the string NULL terminated
      }
    }
  }

}
