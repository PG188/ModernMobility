/* LEFT MOTOR CONTROL MODULE */
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
#include <Filters.h>
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
const double K_P = 380, 
             K_I = 50,
             K_D = 20,
             OUT_MIN = -255,  
             OUT_MAX = 255;
const int SAMPLE_TIME = 10;
bool pOnE = true;

//Serial Constants   
const byte START_FLAG = 0x7F,
           STOP_FLAG = 0x7E;
           
const float radius = 0.132/2; //in meters

//Variables
double lastInput = 0;
unsigned long PID_lastTime = 0;
double outputSum = 0;
double Output, Kp_calc, error, dInput, Ki_calc;
double deadband = 0.075;

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
int period = 0;

double pot;

Encoder myEncoder(ENC1, ENC2);
//float filterFrequency_KD = 1;
//FilterOnePole lowPassFilter_KD(LOWPASS, filterFrequency_KD);

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
        if ((motorVelCmd == -1) || (motorVelCmd == 1)){
          motorVelCmd = 0;
        }
        break;
    }
  }
  //pot = analogRead(2);
  //Serial.println(pot);
  /*if (count > 500){
      motorVelCmd = 0.2;
    }
    else {
      motorVelCmd = -0.2;
  }*/

  /*if (pot > 782) {
    motorVelCmd = -0.1;
  }
  else{
    motorVelCmd = 0;
  }*/
  
  //UPDATE ENCODER
  encoderVal = myEncoder.read();
  encPosition = encoderVal;
  currentTime = (unsigned int)millis();
  if (currentTime - lastTime >= SAMPLE_DELAY) {
    RPM = ((lastPosition-encPosition) * (60000.f / (currentTime - lastTime))) / PULSES_PER_TURN/4;
    wheelVel = convertToLinearVel(RPM);
    lastTime = currentTime;
    lastPosition = encPosition;
    encPosition = 0;
  }

  //DO PID
  /* Graphing wheel velcoity */
  Serial.print(encoderVal);Serial.println(" ");
  //Serial.println(motorVelCmd);

  unsigned long now = millis();
  if(now - PID_lastTime >= SAMPLE_TIME){
    /*Compute all the working error variables*/
    error = (double) motorVelCmd - wheelVel;
    dInput = (double) (wheelVel - lastInput);
    //lowPassFilter_KD.input(dInput);
    if (abs(error) <= deadband){
      Ki_calc = 0;
    }
    else{
      Ki_calc = K_I * error;
    }
    outputSum+= Ki_calc;
    if(outputSum > OUT_MAX) outputSum = OUT_MAX;
    else if(outputSum < OUT_MIN) outputSum = OUT_MIN;

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
    //Output += outputSum - K_D * (double)lowPassFilter_KD.output();
    Output += outputSum - K_D * dInput;
    if(Output > OUT_MAX) Output = OUT_MAX;
    else if(Output < OUT_MIN) Output = OUT_MIN;
    MotorCmd = Output;
    /*Remember some variables for next time*/
    lastInput = wheelVel;
    PID_lastTime = now;
  }

  //Serial.println(MotorCmd);
  digitalWrite(DIR1, signPos(MotorCmd) ? HIGH : LOW);   //Assigning appropriate motor direction
  analogWrite(PWM1,abs(MotorCmd));                      //Actuate motor command
  
  //Send updated encoder value
  if (sendEncoder == 1) {
    Serial.write(byte(encoderVal & 0x00FF)); 
    Serial.write(byte((encoderVal >> 8) & 0x00FF));
  }
  count++;
  delay(30);
}

/*******************************************************************************************************/

bool signPos(int value) {
  return (value >= 0); 
}

float convertToLinearVel(float rpm){
  return rpm*(2*M_PI/60.0)*radius;
}


