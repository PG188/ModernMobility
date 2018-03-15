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
//#define DIR2 7
//#define PWM2 8

//Encoder Pins
#define ENC1 9
#define ENC2 10

//Encoder Parameters
#define SAMPLE_DELAY (10)
#define PULSES_PER_TURN (512)

//Constants
const double K_P = 10, 
             K_I = 10,   //K values must be >= 0
             K_D = 0.1,
             OUT_MIN = -255,  
             OUT_MAX = 255,
             RAMP_DOWN_CMD = 100; 
const int SAMPLE_TIME = 20; 
const byte START_FLAG = 0x7F,
           STOP_FLAG = 0x7E;
const float radius = 0.1143; //in meters

//Variables
double PID_Input,              //The variable we are trying to control (from Motor Module)
       PID_Output,             //The variable that will be adjusted by the PID
       PID_SetPoint;    //The value we are trying to get to or maintain (from ROS node)
float motorVelCmd = 0;
int MotorCmd,
    lastMotorCmd = 0, 
    lastMotorDir = 0;
byte LSB,
     sendEncoder = 0;
int byteRead = 0;
unsigned int lastTime, currentTime; 
int lastPosition = 0;
float RPM, wheelVel;
long encoderVal;
long encPosition  = 0;

int encoderCount = 0;


float filterFrequency_encoder = 0.05;
FilterOnePole lowPassFilter_encoder(LOWPASS, filterFrequency_encoder);
PID MotorPID(&PID_Input, &PID_Output, &PID_SetPoint, K_P, K_I, K_D, DIRECT);
Encoder myEncoder(ENC1, ENC2);

void initPID(){
    MotorPID.SetMode(AUTOMATIC);                //Turns the PID on, default is off (MANUAL)
    MotorPID.SetOutputLimits(OUT_MIN, OUT_MAX); //By default this is (0, 255)
    MotorPID.SetSampleTime(SAMPLE_TIME);        //By default this is 200ms
    MotorPID.SetControllerDirection(DIRECT);
}
void setup() {
    Serial.begin(115200); // Starts the serial communication at 57600 baud (this is fast enough)

    initPID();

    //Motor Outputs
    pinMode(DIR1, OUTPUT);  //Wheel motor direction
    pinMode(PWM1, OUTPUT);  //Wheel motor PWM
    //pinMode(DIR2, OUTPUT);  //Brake motor direction
    //pinMode(PWM2, OUTPUT);  //Brake motor PWM
    //pinMode(0,INPUT);

    //digitalWrite(DIR1, lastMotorDir); //direcitons  
    //digitalWrite(DIR2, 0);
    //digitalWrite(PWM1, lastMotorCmd);   //speed scale speed here with input from pi
    //digitalWrite(PWM2, 0);
}

void loop() {  
    // Receive new motor command or stop/start
    //sendFlag = 0;
    /*if (Serial.available()) {
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
          /*if (byteRead & 0x0080) { //Is byteRead supposed to be negative?
              byteRead = byteRead | 0xFF00; //Sign extends byteRead to 16 bits
          }
          motorVelCmd = float(byteRead)/100; //Converts cm/s value to m/s
          break;
      }  
    }*/
    //motorVelCmd = 0;
    MotorCmd = 100;
    //UPDATE ENCODER
    encoderVal = myEncoder.read();
    //Serial.print("Encoder = ");Serial.print(encoderVal);
    //lowPassFilter_encoder.input(encoderVal);
    //encPosition = lowPassFilter_encoder.output();

    //if (encoderCount >= 5){
    currentTime = (unsigned int)millis();
      //if (currentTime - lastTime >= SAMPLE_DELAY) {
        Serial.print("lastPosition = ");Serial.print(lastPosition);Serial.print(" encoderVal = ");Serial.print(encoderVal);
        Serial.print("  currentTime = ");Serial.print(currentTime);Serial.print("  lastTime = ");Serial.println(lastTime);
        RPM = (60000.f * (lastPosition-encoderVal)) / ((currentTime-lastTime)*PULSES_PER_TURN);
        wheelVel = convertToLinearVel(RPM);
        lastTime = currentTime;
        lastPosition = encoderVal;
        encoderVal = 0;
        //Serial.print(" RPM = ");Serial.println(RPM);
      //}
   //}
    
    //DO PID
    PID_Input = wheelVel;
    PID_SetPoint = (double) motorVelCmd;
    //MotorCmd = doPID(lastMotorCmd);

    lastMotorCmd = MotorCmd;                              //Set previous motor command to current motor command
    
    digitalWrite(DIR1, signPos(MotorCmd) ? HIGH : LOW);   //Assigning appropriate motor direction
    analogWrite(PWM1,abs(MotorCmd));                      //Actuate motor command

    /*
    Serial.println(encPosition);
    Serial.print(" "); //For plotting purposes*/
     
    //Send updated encoder value
    if (sendEncoder == 1) {
      Serial.write(byte(encoderVal & 0x00FF)); 
      Serial.write(byte((encoderVal >> 8) & 0x00FF));
      //Serial.write(0x0000);
    }
    delay(1);
}

int doPID(int previousMotorCmd){
    int newMotorCmd;
    /*
     * MotorPID.Compute() computes the PID and returns true, 
     * if it does not compute anything it will return false
     */
     if (!MotorPID.Compute()) {
      MotorCmd = 0.0;  //Send invalid value when nothing computed
     }
     else {
      newMotorCmd = (int)PID_Output;
      //Serial.println(Output);
      //Serial.print(" "); //For plotting purposes*/
      if (newMotorCmd >= 255){
        newMotorCmd = 255;
      }else if (newMotorCmd < -255){
        newMotorCmd = -255;
      }
     }
     return MotorCmd = newMotorCmd;
     //return MotorPID.Compute() ? (previousMotorCmd - (int)Output) : NULL;
     //MotorPID.Compute();
     //MotorCmd = Output;
}

bool signPos(int value) {
  return (value >= 0); 
}

float convertToLinearVel(float rpm){
  return rpm*(2*M_PI/60.0)*radius;
}
