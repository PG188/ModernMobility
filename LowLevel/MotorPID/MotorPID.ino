#include <Arduino.h>
#include <PID_v1.h>    //Library details: https://playground.arduino.cc/Code/PIDLibrary

#define InputPin 0
#define SetPointPin 1
#define OutputPin 3

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
    Serial.begin(9600);
    initPID();
    Input = 22.0; //Testing purposes 
}

void loop(){
  //Input = analogRead(InputPin);
  //SetPoint_float = analogRead(SetPointPin);
  //SetPoint_double = (double) SetPoint_float;
  SetPoint_double = 20.0; //Testing purposes  
  MotorCmd = doPID();
  // For testing without motors:
  Input = Input + (Output - 128)*2.0/255.0;
  /*Serial.println(Input);
  Serial.print(" "); //For plotting purposes*/

  int MTRSpd = (MotorCmd % 100);  //last two digits
  int MTRdir = (MotorCmd / 100) % 10; // first digit
  Serial.println(MTRSpd);
  Serial.println(MTRdir);
  leftDrive(MTRdir, MTRSpd);        //assuming this is the left motor arduino (change to rightDrive)
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

void leftDrive(int dir, int vel) {        // LEFT MOTOR DRIVE
  int x = map(vel,0,99,0,255);    //scale 0 99 to 0 to 255
  if (dir == 2){
    digitalWrite(DIR1, HIGH); 
  }  
  if (dir == 1){
    digitalWrite(DIR1, LOW); 
  }
  analogWrite(PWM1, x);   //speed scale speed here with input from pi
}


void rightDrive(int dir, int vel){       //RIGHT MOTOR DRIVE
  int x = map(vel,0,99,0,255);    //scale 0 99 to 255
  if (dir == 2){
    digitalWrite(DIR1, HIGH); 
  }  
  if (dir == 1){
    digitalWrite(DIR1, LOW); 
  }
  analogWrite(PWM2, x);   //speed scale speed here with input from pi
}

