/*
This software reads the value from each of the ultrasonic sensors and manual slider.
Every 10ms these values are sent back to the Raspberry Pi for processing 
*/


/*
Issues: The stop reset is not working. Could stem from the bytes potentially ariving after the 
first block of code with the Serial.available check. How can we make it so this cannot happen?
*/

#include <Arduino.h>
#include <PID_v1.h>    //Library details: https://playground.arduino.cc/Code/PIDLibrary

//Motor Pins
#define DIR1 6
#define PWM1 5
#define DIR2 4
#define PWM2 3
#define velocityInPin 1

//Constants
const double K_P = 5, 
             K_I = 2,   //K values must be >= 0
             K_D = 0,
             OUT_MIN = 0,  
             OUT_MAX = 255,
             RAMP_DOWN_CMD = 100; 
const int SAMPLE_TIME = 20; 
const byte START_FLAG = 0x7F,
           STOP_FLAG = 0x7E;

//Variables
double Input,              //The variable we are trying to control (from Motor Module)
       Output,             //The variable that will be adjusted by the PID
       SetPoint_double;    //The value we are trying to get to or maintain (from ROS node)
float motorVelCmd = 0;
int MotorCmd,
    initMotorCmd = 0, 
    initMotorDir = 0;
byte LSB,
     sendEncoder = 0;
int encoder = 1.8;  //ENCODER CODE GOES HERE
int byteRead = 0;

//Creating PID
PID MotorPID(&Input, &Output, &SetPoint_double, K_P, K_I, K_D, DIRECT);

void setup() {
    Serial.begin(115200); // Starts the serial communication at 57600 baud (this is fast enough)
    delay(2000);

    //initPID();

    //Encoder Input
    pinMode(velocityInPin, INPUT);

    //Motor Outputs
    pinMode(DIR1, OUTPUT);  //Wheel motor direction
    pinMode(PWM1, OUTPUT);  //Wheel motor PWM
    pinMode(DIR2, OUTPUT);  //Brake motor direction
    pinMode(PWM2, OUTPUT);  //Brake motor PWM

    digitalWrite(DIR1, initMotorCmd); //direcitons  
    digitalWrite(DIR2, 0);
    analogWrite(PWM1, initMotorCmd);   //speed scale speed here with input from pi
    analogWrite(PWM2, 0);
}

void loop() {
  //Serial.println(Input);
  //Serial.print(" ");
    //UPDATE ENCODER
    // Receive new motor command or stop/start
    if (Serial.available()) {
      byteRead = Serial.read();
      switch (byteRead){
        case START_FLAG: sendEncoder = 1; break;
        case STOP_FLAG: sendEncoder = 0; break;
        default:  //Velocity command
          /*If byteRead is negative, its MSB will be 1 because 2s compliment. 
          * However, because arduino is dumb, it reads in bytes as 16-bit integers,
          * So, when you try to read the value, its 8 MSB are zero, making
          * Arduino think it is a positive number. We must set these bits to 1
          * (sign extension) to get it to recognize byteRead as negative.
          */
          if (byteRead & 0x0080) { //Is byteRead supposed to be negative?
              byteRead = byteRead | 0xFF00; //Sign extends byteRead to 16 bits
          }
          motorVelCmd = float(byteRead)/50.0; //Converts cm/s value to m/s
          break;
      }
     }
    
    //initMotorCmd = MotorCmd;                              //Set previous motor command to current motor command
    MotorCmd = motorVelCmd*255;
    if (signPos(MotorCmd)){
      digitalWrite(DIR1, HIGH);   //Assigning appropriate motor direction
    }
    if (signNeg(MotorCmd)){
      digitalWrite(DIR1, LOW);
    }
    analogWrite(PWM1,abs(MotorCmd));                      //Actuate motor command
  
    /*Serial.println(Input);
    Serial.print(" "); //For plotting purposes*/
     
    //Send updated encoder value
    if (sendEncoder == 1) {
      Serial.write(byte(encoder & 0x00FF)); 
      Serial.write(byte((encoder >> 8) & 0x00FF));
    }
    delay(30);
}

bool signPos(int value) {
  return (value >= 0); 
}

bool signNeg(int value) {
  return (value < 0); 
}
