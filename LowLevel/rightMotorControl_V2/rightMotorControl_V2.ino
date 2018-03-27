/* RIGHT MOTOR CONTROL MODULE */
//ModernMobility

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
#include <Encoder.h>
#define ENCODER_OPTIMIZE_INTERRUPTS

//Motor Pins
#define DIR1 6
#define PWM1 5

//Encoder Pins
#define ENC1 3
#define ENC2 4

//Encoder Parameters
#define SAMPLE_DELAY 10
#define PULSES_PER_TURN 512

/*  Constants */
//PID Constants
const double K_P = 450, 
             K_I = 100,
             K_D = 1,
             OUT_MIN = -255,  
             OUT_MAX = 255;
const int SAMPLE_TIME = 30;
bool pOnE = true;

//Serial Constants   
const byte START_FLAG = 0x7F,
           STOP_FLAG = 0x7E;
           
const float radius = 0.12065; //in meters

//Variables
//double PID_Input,              //The variable we are trying to control (from Motor Module)
//       PID_Output,             //The variable that will be adjusted by the PID
//       PID_SetPoint;    //The value we are trying to get to or maintain (from ROS node)
double lastInput = 0;
unsigned long PID_lastTime = 0;
double outputSum = 0;
double Output, Kp_calc;
double error;
double dInput;
double deadband = 0.05;
double Ki_calc;


//PID values
float motorVelCmd = 0;
int MotorCmd = 0,
    initMotorCmd = 0, 
    initMotorDir = 0,
    lastMotorCmd = 0;

//Serial reading values
int byteRead = 0,
    sendEncoder = 0;
    
//Encoder reading values
unsigned int lastTime = 0, currentTime = 0; 
long lastPosition = 0;
float RPM = 0, wheelVel = 0;
int encoderVal = 0;
int encPosition  = 0;

unsigned int wheelVelOut;
int count = 0;
int something = 0;

Encoder myEncoder(ENC1, ENC2);

void setup() {
    Serial.begin(115200); // Starts the serial communication at 57600 baud (this is fast enough)
    //initPID();
    //Motor Outputs
    pinMode(DIR1, OUTPUT);  //Wheel motor direction
    pinMode(PWM1, OUTPUT);  //Wheel motor PWM

    digitalWrite(DIR1, HIGH); //direcitons  
    digitalWrite(PWM1, initMotorCmd);   //speed scale speed here with input from pi
}

void loop() {  
  // Receive new motor command or stop/start
  //sendFlag = 0;
  if (Serial.available()){
    //count = 0; 
    byteRead = Serial.read();
    //byteString = Serial.readString();
    //Serial.print("Num bytes available: ");Serial.println(Serial.available());
    //Serial.print("I received: ");
    //Serial.println(byteRead);
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
        motorVelCmd = -float(byteRead)/100; //Converts cm/s value to m/s
        break;
    }
  }

  /*if (count > 1000){
    motorVelCmd = 0.2;
  }
  else {
    motorVelCmd = 0.2;
  }*/
    
  //UPDATE ENCODER
  encoderVal = myEncoder.read();
  encPosition = encoderVal;
  currentTime = (unsigned int)millis();
  if (currentTime - lastTime >= SAMPLE_DELAY) {
    RPM = ((encPosition-lastPosition) * (60000.f / (currentTime - lastTime))) / PULSES_PER_TURN/4;
    wheelVel = convertToLinearVel(RPM);
    lastTime = currentTime;
    lastPosition = encPosition;
    encPosition = 0;
  }
  
  //DO PID
  /* Graphing wheel velcoity */
  //Serial.print(encoderVal);Serial.println(" ");
  //Serial.println(motorVelCmd);

  unsigned long now = millis();
  if(now - PID_lastTime >= SAMPLE_TIME){
    /*Compute all the working error variables*/
    //Serial.print("SetPoint = ");Serial.print(motorVelCmd);Serial.print("  Input = ");Serial.print(wheelVel);
    error = (double) motorVelCmd - wheelVel;
    //Serial.print("  Error = ");Serial.print(error);
    dInput = (double) (wheelVel - lastInput);
    //Serial.print("  dInput = ");Serial.print(dInput);
    if (abs(error) <= deadband){
      Ki_calc = 0;
    }
    else{
      Ki_calc = K_I * error;
    }
    outputSum+= Ki_calc;
    //Serial.println(*outputSum);
    if(outputSum > OUT_MAX) outputSum = OUT_MAX;
    else if(outputSum < OUT_MIN) outputSum = OUT_MIN;
    //Serial.print("  OutputSum = ");Serial.print(outputSum);

    /*Add Proportional on Error, if P_ON_E is specified*/
    if(pOnE){
      if (abs(error) <= deadband){
        Kp_calc = 0;
      }
      else{
        Kp_calc = K_P * error;
      }
      Output = Kp_calc;
      }
    else Output = 0;

    /*Compute Rest of PID Output*/
    Output += outputSum - K_D * dInput;
    //Serial.println(Output);
    if(Output > OUT_MAX) Output = OUT_MAX;
    else if(Output < OUT_MIN) Output = OUT_MIN;
    //Serial.print("  Output = ");Serial.println(Output);
    MotorCmd = Output;

      /*Remember some variables for next time*/
    lastInput = wheelVel;
    PID_lastTime = now;
  }
  MotorCmd = 0; 
  digitalWrite(DIR1, signPos(MotorCmd) ? HIGH : LOW);   //Assigning appropriate motor direction
  analogWrite(PWM1,abs(MotorCmd));                      //Actuate motor command
  
  encoderVal = -encoderVal;  //Send updated encoder value
  if (sendEncoder == 1) {
    Serial.write(byte(encoderVal & 0x00FF)); 
    Serial.write(byte((encoderVal >> 8) & 0x00FF));
  }
  //count++;
  delay(30);
}

/*******************************************************************************************************/

bool signPos(int value) {
  return (value >= 0); 
}

float convertToLinearVel(float rpm){
  return rpm*(2*M_PI/60.0)*radius;
}