#include <Arduino.h>
#include <PID_v1.h>    //Library details: https://playground.arduino.cc/Code/PIDLibrary

#define InputPin 0
#define SetPointPin 1
#define OutputPin 3

//Constants
const double K_P = 1, 
             K_I = 0.25,   //K values must be >= 0
             K_D = 0,
             OUT_MIN = 0,    //-99 (+ 99) 0
             OUT_MAX = 99;  //+99 (+ 99) 198

const int SAMPLE_TIME = 20; 

//Variables
double Input,      //The variable we are trying to control
       SetPoint,   //The value we are trying to get to or maintain
       Output;     //The variable that will be adjusted by the PID

//PID Object
//PID(&double, &double, &double, double, double, double, Direction)
PID MotorPID(&Input, &Output, &SetPoint, K_P, K_I, K_D, DIRECT);

/**************************************************************/

void setup(){
    Serial.begin(9600);
    initPID();
    Input = 10; //Testing purposes 
}

void loop(){
  //Input = analogRead(InputPin);
  SetPoint = 20;  //SetPoint = analogRead(SetPointPin); 
  doPID();
  Serial.println(Input);
  Serial.print(" ");
  //Serial.println(SetPoint);
  // For testing without motors:
  if (Input < SetPoint) {
    Input = Input + Output*2.0/99.0;
  }
  else if (Input > SetPoint) {
    Input = Input - Output*2.0/99.0;
  }
  //analogWrite(#, Output);
}

/**************************************************************/

void initPID(){
    MotorPID.SetMode(AUTOMATIC);                //Turns the PID on, default is off (MANUAL)
    MotorPID.SetOutputLimits(OUT_MIN, OUT_MAX); //By default this is (0, 255)
    MotorPID.SetSampleTime(SAMPLE_TIME);        //By default this is 200ms
}

void doPID(){
    /*
     * MotorPID.Compute() computes the PID and returns true, 
     * if it does not compute anything it will return false
     */
    if (!MotorPID.Compute()) {
        //Serial.println("PID returned false");
    }
    else {
      //Serial.print("Output Velocity = "); Serial.println(Output);
    }
}


