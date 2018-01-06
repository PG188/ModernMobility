//ma'fuck pins
#include <Wire.h>
#include <stdint.h>
#define SLAVE_ADDRESS 0x04 //ar grn A4         yellow A5        brn GRND
                           //pi green gpoio8   yellow gpoio9     brn gpoio6
int DIR1 = 6; //digitial
int PWM1 = 5;
int DIR2 = 4;
int PWM2 = 3;

int Power=0;

uint32_t number = 0;
int state = 0;

//ma'fuck setup///////////////////////////////////////////////////////////////////////////
void setup() {
  pinMode(13, OUTPUT);
  
  pinMode(DIR1, OUTPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM2, OUTPUT);
  
  Serial.begin(9600);
  
  Wire.begin(SLAVE_ADDRESS);
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("Ready!");

  digitalWrite(DIR1, 0); //direcitons  
  digitalWrite(DIR2, 0);
  analogWrite(PWM1, 0);   //speed scale speed here with input from pi
  analogWrite(PWM2, 0);
}


//ma'fuck code/////////////////////////////////////////////////////////////////////////////////

void loop() { 
  delay(100);
}


//ma'fuck funcitons//////////////////////////////////////////////////////////////////////////////

void receiveData(int byteCount) {   //read data
  while (Wire.available()) {
  number = Wire.read();           // recieve i2c signal in the form "012-345" ignore dash 
  
  int num5 = (number) % 10;       //seperate numbers into digits
  int num4 = (number / 10) % 10;
  int num3 = (number / 100) % 10;
  int num2 = (number / 1000) % 10;
  int num1 = (number / 10000) % 10; 
  int num0 = (number / 100000) % 10; 
  
  // left forward half    = 101-005
  // right forwards full  = 110-090
  // both backwards full  = 100-099
  // both forwards quarter= 111-022
  
  leftDrive(num2, num5);
  rightDrive(num1, num4); 
  Serial.println(number);
  }
}  

void sendData() {                 // callback for sending data
  Wire.write(number);
}


void leftDrive(int dir, int vel) {
  int x = map(vel,1,9,0,255);
  digitalWrite(DIR1, dir); //direcitons  
  analogWrite(PWM1, x);   //speed scale speed here with input from pi
}

void rightDrive(int dir, int vel){
  int x = map(vel,0,9,0,255);
  digitalWrite(DIR2, dir);
  analogWrite(PWM2, x);
}



void Fade (int PWM, int fadeAmount){  //test cyclical power
  analogWrite(PWM, Power);

  Power = Power + fadeAmount;
  
  if (Power <= 0 || Power >= 255) {
    fadeAmount = -fadeAmount;
  }
  delay(150);
}



///////////////////////////////////extra/////////////////////////////////

  
 /* 
  if (number == 1){             //led test
   if (state == 0){
    digitalWrite(13, HIGH); // set the LED on
    state = 1;
   } else{
    digitalWrite(13, LOW); // set the LED off
    state = 0;
   }
  }
*/


