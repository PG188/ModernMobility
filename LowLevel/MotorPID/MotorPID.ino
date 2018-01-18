#include <Arduino.h>
#include <PID_v1.h>    //Library details: https://playground.arduino.cc/Code/PIDLibrary

<<<<<<< HEAD
//Motor Pins
#define DIR1 6
#define PWM1 5
#define DIR2 4
#define PWM2 3
#define velocityInPin 1
=======
#define velocityInPin 0
#define velocitySetPointPin 1
#define velocityOutPin 3
#define BrakePin 4

//Pins
int DIR1 = 6; //PINOUT digitial pin
int PWM_L = 5; //left spd
int DIR2 = 4;
int PWM_R = 3; //right spd
>>>>>>> 1904633e30af013a137cb15d5128c98cfaf70302

//Constants
const double K_P = 5, 
             K_I = 2,   //K values must be >= 0
             K_D = 0,
             OUT_MIN = -255,    //-99 (+ 99) 0
             OUT_MAX = 255,  //+99 (+ 99) 198
             MAG_MAX = 99;
const int SAMPLE_TIME = 20; 

//Enums
enum MOTOR_CMD_DIR { POS = 200, NEG = 100 };

//Variables
double Input,              //The variable we are trying to control (from Motor Module)
       Output,             //The variable that will be adjusted by the PID
       SetPoint_double;    //The value we are trying to get to or maintain (from ROS node)
int MotorCmd;
int initMotorCmd = 0, initMotorDir = 0;
byte LSB;


//Creating PID
PID MotorPID(&Input, &Output, &SetPoint_double, K_P, K_I, K_D, DIRECT);

/**************************************************************/

void setup(){
    Serial.begin(9600);
    initPID();

<<<<<<< HEAD
    //Encoder Input
    pinMode(velocityInPin, INPUT);

    //Motor Outputs
    pinMode(DIR1, OUTPUT);  //Wheel motor direction
    pinMode(PWM1, OUTPUT);  //Wheel motor PWM
    pinMode(DIR2, OUTPUT);  //Brake motor direction
    pinMode(PWM2, OUTPUT);  //Brake motor PWM
=======
    pinMode(DIR1, OUTPUT); //motor outputs
    pinMode(PWM_L, OUTPUT);
    pinMode(DIR2, OUTPUT);
    pinMode(PWM_R, OUTPUT);
>>>>>>> 1904633e30af013a137cb15d5128c98cfaf70302

    digitalWrite(DIR1, initMotorCmd); //direcitons  
    digitalWrite(DIR2, 0);
<<<<<<< HEAD
    analogWrite(PWM1, initMotorCmd);   //speed scale speed here with input from pi
    analogWrite(PWM2, 0);
=======
    analogWrite(PWM_L, 0);   //speed scale speed here with input from pi
    analogWrite(PWM_R, 0);
>>>>>>> 1904633e30af013a137cb15d5128c98cfaf70302
}

void loop(){
  //SetPoint_double = 20.0; //Testing purposes 
  Input = analogRead(velocityInPin);  //Encoder value
  LSB = Serial.read();
  SetPoint_double = BitShiftCombine(Serial.read(),LSB);
  if (SetPoint_double == 999) { //Ramp-down speed
    if (Input != 0) {
      analogWrite(PWM2, 0); //Disengage clutch
      SetPoint_double = 0;      //Set the Set Point to 0
      MotorCmd = doPID(initMotorCmd);
    }
    else {
      delay(2000);
      analogWrite(PWM2, 255); //Engage clutch
    }
  }
  else { 
    MotorCmd = doPID(initMotorCmd);
  }
  initMotorCmd = MotorCmd;
  if (signPos(MotorCmd)) {digitalWrite(DIR1, HIGH);}
  else {analogWrite(PWM1, abs(MotorCmd));}
  
  /*Serial.println(Input);
  Serial.print(" "); //For plotting purposes*/
<<<<<<< HEAD
  //leftDrive(MTRdir, MTRSpd);        //assuming this is the left motor arduino (change to rightDrive)
=======

  int MTRSpd = (MotorCmd % 100);  //last two digits
  int MTRdir = (MotorCmd / 100) % 10; // first digit
  Serial.println(MTRSpd);
  Serial.println(MTRdir);
  drive(MTRdir, MTRSpd, PWM_L); //3rd param is PWM_L for left motor arduino & PWM_R for right
>>>>>>> 1904633e30af013a137cb15d5128c98cfaf70302
}

/**************************************************************/

void initPID(){
    MotorPID.SetMode(AUTOMATIC);                //Turns the PID on, default is off (MANUAL)
    MotorPID.SetOutputLimits(OUT_MIN, OUT_MAX); //By default this is (0, 255)
    MotorPID.SetSampleTime(SAMPLE_TIME);        //By default this is 200ms
    MotorPID.SetControllerDirection(DIRECT);
}

double BitShiftCombine( unsigned char x_high, unsigned char x_low)
{
  int combined; 
  combined = x_high;              //send x_high to rightmost 8 bits
  combined = combined<<8;         //shift x_high over to leftmost 8 bits
  combined |= x_low;                 //logical OR keeps x_high intact in combined and fills in                                                             //rightmost 8 bits
  return (double) combined;
}

int doPID(int previousMotorCmd){
    /*
     * MotorPID.Compute() computes the PID and returns true, 
     * if it does not compute anything it will return false
     */
    if (!MotorPID.Compute()) {
        //Serial.println("PID returned false");
        return NULL;  //Send invalid value when nothing computed
    }
    else {
<<<<<<< HEAD
      return (previousMotorCmd - (int)Output);
=======
      return DetermineMotorCmd(Input, Output); 
>>>>>>> 1904633e30af013a137cb15d5128c98cfaf70302
    }
}

bool signPos(int value) { 
 if (value >= 0){ return true; }
 else {return false;}
}

/*int DetermineMotorCmd( double Input, double Output){
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
<<<<<<< HEAD
  analogWrite(PWM1, x);   //speed scale speed here with input from pi
}*/

=======
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

>>>>>>> 1904633e30af013a137cb15d5128c98cfaf70302


