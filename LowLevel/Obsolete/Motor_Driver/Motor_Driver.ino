//ma'fuck pins
#include <Wire.h>
#define SLAVE_ADDRESS 0x04 //ar grn A4         yellow A5        brn GRND
                           //pi green gpoio8   yellow gpoio9     brn gpoio6


int DIR1 = 6; //PINOUT digitial pin
int PWM1 = 5; //left spd
int DIR2 = 4;
int PWM2 = 3; //right spd

int Power=0;

int number = 0;
int state = 0;
int32_t value = 0;

//ma'fuck setup///////////////////////////////////////////////////////////////////////////
void setup() {
  //pinMode(13, OUTPUT);
  
  pinMode(DIR1, OUTPUT); //motor outputs
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
  value = 299;  // forward full speed
}


//ma'fuck code/////////////////////////////////////////////////////////////////////////////////

void loop() {  
    int MTRSpd = (value % 100);  //last two digits
    int MTRdir = (value / 100) % 10; // first digit
    Serial.println(MTRSpd);
    Serial.println(MTRdir);
    leftDrive(MTRdir, MTRSpd);
}


//ma'fuck funcitons//////////////////////////////////////////////////////////////////////////////


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





void receiveData(int byteCount) {   //read data
  while (Wire.available()) {

    if (state == 0){                //change number read thoruhg i2c into computable value 
      value = 0;
      number = Wire.read();         // recieve i2c signal in the form "012-345" ignore dash
      value= number*100;            // multiply by 100 to value to create  "012-000"
      state++;
    }
    if (state == 1){
      number = Wire.read();         // recieve i2c signal in the form "012-345" ignore dash
      value= value + number;        // add read value to value to create  "012-345"
      state++;
    }
    
    int num5 = (value) % 10;       //seperate numbers into digits
    int num4 = (value / 10) % 10;
    int num3 = (value / 100) % 10;
    int num2 = (value / 1000) % 10;
    int num1 = (value / 10000) % 10; 
    int num0 = (value / 100000) % 10; 
    
    // left forward half    = 101-005       this doesnt give the best res possible but it will work
    // right forwards full  = 110-090
    // both backwards full  = 100-099
    // both forwards quarter= 111-022
    
    if (state = 2){
    Serial.println(value);
    leftDrive(num2, num5);
    rightDrive(num1, num4); 
    state = 0;
    }
  }
}  

void sendData() {                 // callback for sending data
  Wire.write(number);
}


void Fade (int PWM, int fadeAmount){  //test cyclical power
  analogWrite(PWM, Power);

  Power = Power + fadeAmount;
  
  if (Power <= 0 || Power >= 255) {
    fadeAmount = -fadeAmount;
  }
  delay(150);
}
