#include <Arduino.h>
#include <PID_v1.h>    //Library details: https://playground.arduino.cc/Code/PIDLibrary

#define velocityInPin 0
#define velocitySetPointPin 1
#define velocityOutPin 3
#define BrakePin 4

//Pins
int DIR1 = 6; //PINOUT digitial pin
int PWM_L = 5; //left spd
int DIR2 = 4;
int PWM_R = 3; //right spd

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

    pinMode(DIR1, OUTPUT); //motor outputs
    pinMode(PWM_L, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM_R, OUTPUT);

    digitalWrite(DIR1, 0); //direcitons  
    digitalWrite(DIR2, 0);
    analogWrite(PWM_L, 0);   //speed scale speed here with input from pi
    analogWrite(PWM_R, 0);
}

void loop(){
  //SetPoint_double = 20.0; //Testing purposes 
  Input = analogRead(velocityInPin);
  SetPoint_float = analogRead(velocitySetPointPin);
  SetPoint_double = (double) SetPoint_float;
  if (SetPoint_double == 999) { //Ramp-down speed
    if (Input != 0) {
      digitalWrite(BrakePin, 0); //Disengage clutch
      SetPoint_double = 0;      //Set the Set Point to 0
      MotorCmd = doPID();
    }
    else {
      delay(2000);
      analogWrite(BrakePin, 1); //Engage clutch
    }
  }
  else { 
    MotorCmd = doPID();
  }
  /*Serial.println(Input);
  Serial.print(" "); //For plotting purposes*/

  int MTRSpd = (MotorCmd % 100);  //last two digits
  int MTRdir = (MotorCmd / 100) % 10; // first digit
  Serial.println(MTRSpd);
  Serial.println(MTRdir);
  drive(MTRdir, MTRSpd, PWM_L); //3rd param is PWM_L for left motor arduino & PWM_R for right
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
        return NULL;  //Send invalid value when nothing computed
    }
    else {
      return DetermineMotorCmd(Input, Output); 
    }
}
int DetermineMotorCmd( double Input, double Output){
    int shiftedVal = int(Output - MAG_MAX);
    if (shiftedVal > MAG_MAX){ return ((int)POS + (int)MAG_MAX); }
    else if (shiftedVal >= 0) { return ((int)POS + shiftedVal); }
    else if (shiftedVal >= -1*MAG_MAX) { return ((int)NEG + abs(shiftedVal)); }
    else { return (NEG + MAG_MAX); }
}

void drive(int dir, int vel, int PWM){       //RIGHT MOTOR DRIVE
  int x = map(vel,0,99,0,255);    //scale 0 99 to 255
  switch (dir){
    case 2: digitalWrite(DIR1, HIGH); break;
    case 1: digitalWrite(DIR1, LOW); break;
    default: break;
  }
  analogWrite(PWM, x);   //speed scale speed here with input from pi
}

/*
void leftDrive(int dir, int vel) {        // LEFT MOTOR DRIVE
  int x = map(vel,0,99,0,255);    //scale 0 99 to 0 to 255
  if (dir == 2){
    digitalWrite(DIR1, HIGH); 
  }  
  if (dir == 1){
    digitalWrite(DIR1, LOW); 
  }
  analogWrite(PWM_L, x);   //speed scale speed here with input from pi
}

void rightDrive(int dir, int vel){  //RIGHT MOTOR DRIVE
  int x = map(vel,0,99,0,255);      //scale 0 99 to 255
  if (dir == 2){
    digitalWrite(DIR1, HIGH); 
  }  
  if (dir == 1){
    digitalWrite(DIR1, LOW); 
  }
  analogWrite(PWM_R, x);   //speed scale speed here with input from pi
}
*/



