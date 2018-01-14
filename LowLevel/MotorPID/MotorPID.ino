#include <Arduino.h>
#include <PID_v1.h>    //Library details: https://playground.arduino.cc/Code/PIDLibrary

#define velocityInPin 0
#define velocitySetPointPin 1
#define velocityOutPin 3
#define BrakePin 4

//Constants
const double K_P = 5, 
             K_I = 2,   //K values must be >= 0
             K_D = 0,
             OUT_MIN = 0,    //-99 (+ 99) 0
             OUT_MAX = 198,  //+99 (+ 99) 198
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
    Serial.begin(19200);
    initPID();
    Input = 22.0; //Testing purposes 
}

void loop(){
  SetPoint_double = 20.0; //Testing purposes 
  //Input = analogRead(velocityInPin);
  //SetPoint_float = analogRead(velocitySetPointPin);
  //SetPoint_double = (double) SetPoint_float;
  if (SetPoint_double == 999) { //
    if (Input != 0) {
      digitalWrite(BrakePin, 0); //Disengage clutch
      SetPoint_double = 0;      //Set the Set Point to 0
      MotorCmd = doPID();
      analogWrite(velocityOutPin, MotorCmd);
    }
    else {
      delay(2000);
      analogWrite(BrakePin, 1); //Engage clutch
    }
  }
  else { 
  MotorCmd = doPID();
  // For testing without motors:
  Input = Input + (Output - 128)*2.0/255.0;
  Serial.println(Input);
  Serial.print(" "); //For plotting purposes
  //analogWrite(velocityOutPin, MotorCmd); //Send command to wheel motor
  }
}

/**************************************************************/

void initPID(){
    MotorPID.SetMode(AUTOMATIC);                //Turns the PID on, default is off (MANUAL)
    MotorPID.SetOutputLimits(OUT_MIN, OUT_MAX); //By default this is (0, 255)
    MotorPID.SetSampleTime(SAMPLE_TIME);        //By default this is 200ms
    MotorPID.SetControllerDirection(DIRECT);
}

int doPID(){
    /*
     * MotorPID.Compute() computes the PID and returns true, 
     * if it does not compute anything it will return false
     */
    if (!MotorPID.Compute()) {
        //Serial.println("PID returned false");
    }
    else {
      return DetermineMotorCmd( Input, Output); 
    }
}
int DetermineMotorCmd( double Input, double Output){
    int shiftedVal = int(Output - MAG_MAX);
    if (shiftedVal > MAG_MAX){ return ((int)POS + (int)MAG_MAX); }
    else if (shiftedVal >= 0) { return ((int)POS + shiftedVal); }
    else if (shiftedVal >= -MAG_MAX) { return ((int)NEG + abs(shiftedVal)); }
    else { return (NEG + MAG_MAX); }
}

