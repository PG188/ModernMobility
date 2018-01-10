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
             OUT_MAX = 255;  //+99 (+ 99) 198
             MAG_MAX = 99;
const int SAMPLE_TIME = 20; 

//Enums
enum MOTOR_CMD_DIR { POS = 200, NEG = 100 };

//Variables
float SetPoint_float;      //The value we are trying to get to or maintain (from ROS node)
double Input,              //The variable we are trying to control (from Motor Module)
       Output,             //The variable that will be adjusted by the PID
       SetPoint_double;
int MotorCmd;

//Creating PID
PID MotorPID(&Input, &Output, &SetPoint_double, K_P, K_I, K_D, DIRECT);

/**************************************************************/

void setup(){
    Serial.begin(9600);
    initPID();
    //Input = 10; //Testing purposes 
}

void loop(){
  //Input = analogRead(InputPin);
  //SetPoint_float = analogRead(SetPointPin);
  //SetPoint_double = (double) SetPoint_float;
  //SetPoint_double = 20.0; //Testing purposes  
  doPID();
  // For testing without motors:
  /*if (Input < SetPoint_double) {
    Input = Input + Output*2.0/99.0;
  }
  else if (Input > SetPoint_double) {
    Input = Input - Output*2.0/99.0;
  }*/
  MotorCmd = DetermineMotorCmd(Input, Output);
  //Serial.print("Motor Command = "); 
  Serial.println(MotorCmd);
  Serial.print(" ");
  //analogWrite(OutputPin, MotorCmd);
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
}
int DetermineMotorCmd( double Input, double Output){
    double ratio = MAG_MAX/OUT_MAX;
    int sign, mag;
    sign = (Input >= 0) ? POS : NEG;
    mag = (Output >= OUT_MAX) ? OUT_MAX*ratio : (Output < OUT_MIN) ? OUT_MIN : round(Output*ratio);    
    return sign + mag;
}

