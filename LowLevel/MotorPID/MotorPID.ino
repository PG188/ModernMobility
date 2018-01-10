#include <Arduino.h>
#include <PID_v1.h>    //Library details: https://playground.arduino.cc/Code/PIDLibrary

//Constants
const double K_P = 1, 
             K_I = 1,   //K values must be >= 0
             K_D = 1,
             OUT_MIN = 0,    //-99 (+ 99)
             OUT_MAX = 198;  //+99 (+ 99)

const int SAMPLE_TIME = 200; 

//Variables
double Input = 0,    //The variable we are trying to control
       Output = 0,   //The variable that will be adjusted by the PID
       SetPoint = 0; //The value we are trying to get to or maintain

float currentVel = 0, 
      targetVel = 0; 
      

//PID Object
//PID(&double, &double, &double, double, double, double, Direction)
PID MotorPID(&Input, &Output, &SetPoint, K_P, K_I, K_D, DIRECT);

/**************************************************************/

void setup(){
    initPID();
}

void loop(){
    currentVel = 0; //should = analogRead(#); 
    targetVel = 0;  //should = analogRead(#); 
    
    doPID(currentVel, targetVel);

    //analogWrite(#, Output);
}

/**************************************************************/

void initPID(){
    MotorPID.SetMode(AUTOMATIC);                //Turns the PID on, default is off (MANUAL)
    MotorPID.SetOutputLimits(OUT_MIN, OUT_MAX); //By default this is (0, 255)
    MotorPID.SetSampleTime(SAMPLE_TIME);        //By default this is 200ms
}

float doPID(float input, float setpoint){
    Input = input;
    SetPoint = setpoint;

    /*
     * MotorPID.Compute() computes the PID and returns true, 
     * if it does not compute anything it will return false
     */
    if (!MotorPID.Compute())  
    {
        //<EXECUTE EMERGENCY ACTION>
    }
}


