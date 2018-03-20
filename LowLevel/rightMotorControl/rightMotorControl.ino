
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
const double K_P = 120, 
             K_I = 100,
             K_D = 1,
             OUT_MIN = -210,  
             OUT_MAX = 210;
const int SAMPLE_TIME = 30;

//Serial Constants   
const byte START_FLAG = 0x7F,
           STOP_FLAG = 0x7E;
           
const float radius = 0.12065; //in meters

//Variables
double Input,              //The variable we are trying to control (from Motor Module)
       Output,             //The variable that will be adjusted by the PID
       SetPoint_double;    //The value we are trying to get to or maintain (from ROS node)

//PID values
float motorVelCmd = 0, lastMotorVelCmd = 0;
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
long encoderVal = 0;
long encPosition  = 0;

unsigned int wheelVelOut;
int count = 0;

//Creating PID
PID MotorPID(&Input, &Output, &SetPoint_double, K_P, K_I, K_D, DIRECT);
Encoder myEncoder(ENC1, ENC2);

void setup() {
    Serial.begin(115200); // Starts the serial communication at 57600 baud (this is fast enough)
    initPID();
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
      count = 0; 
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
      motorVelCmd = -0.2;
    }
    else {
      motorVelCmd = 0.2;
    }*/
    
    /*if (count >= 100){
      if (motorVelCmd == lastMotorVelCmd)
        motorVelCmd = 0;
    }
    else{*/
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
      Input = wheelVel;
      SetPoint_double = (double) motorVelCmd;
      MotorCmd = doPID();
      lastMotorCmd = MotorCmd;
      //Serial.print(MotorCmd);Serial.println(" ");
    //}
    digitalWrite(DIR1, signPos(MotorCmd) ? HIGH : LOW);   //Assigning appropriate motor direction
    analogWrite(PWM1,abs(MotorCmd));                      //Actuate motor command
  
    //Serial.println(wheelVel);
    //Serial.print(" "); //For plotting purposes*/
    //wheelVelOut = (unsigned int)abs(byteRead*100);
    
    //Send updated encoder value
    if (sendEncoder == 1) {
      Serial.write(byte(encoderVal & 0x00FF)); 
      Serial.write(byte((encoderVal >> 8) & 0x00FF));
    }
    count++;
    delay(30);
}

/*******************************************************************************************************/

void initPID(){
    MotorPID.SetMode(AUTOMATIC);                //Turns the PID on, default is off (MANUAL)
    MotorPID.SetOutputLimits(OUT_MIN, OUT_MAX); //By default this is (0, 255)
    MotorPID.SetSampleTime(SAMPLE_TIME);        //By default this is 200ms
    MotorPID.SetControllerDirection(DIRECT);
    MotorPID.SetTunings(K_P, K_I, K_D, P_ON_E);
}

int doPID(){
    /*
     * MotorPID.Compute() computes the PID and returns true, 
     * if it does not compute anything it will return false
     */
     double MotorCmd;
     if (!MotorPID.Compute()) {
      return 0;  //Send invalid value when nothing computed
     }
     else {
       //Serial.print("Output = ");Serial.print(Output);
      /*if (signPos(lastOutput) && !signPos(Output)){
        Output = -1;
        delay(10);
      }
      else if (!signPos(lastOutput) && signPos(Output)){
        Output = 0;
        delay(10);
      }*/
      /*if (Output >= -20 && Output <= 20){
          Output = 0;
        }
      else if (Output > 20 && Output <= 45){
          Output = 50;
        }
      else if (Output < -20 && Output >= -45){
        Output = -50;
      }*/
      if (Output > 0)
        MotorCmd = Output + 45;
      else if (Output < 0)
        MotorCmd = Output - 45;
      else
        MotorCmd = 0;
      //Serial.print("  MotorCmd = ");Serial.println(MotorCmd);
      return MotorCmd;
     }
}

bool signPos(int value) {
  return (value >= 0); 
}

float convertToLinearVel(float rpm){
  return rpm*(2*M_PI/60.0)*radius;
}

