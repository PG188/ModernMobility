//ma'fuck pins
#include <Wire.h>

#define SLAVE_ADDRESS 0x04 //ar grn A4         yellow A5        brn GRND
                           //pi green gpoio8   yellow gpoio9     brn gpoio6
int DIR1 = 6; //digitial
int PWM1 = 5;
int DIR2 = 4;
int PWM2 = 3;

int Power=0;

//char number[50];
int number = 0;
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
}


//ma'fuck code/////////////////////////////////////////////////////////////////////////////////

void loop() {

  digitalWrite(DIR1, HIGH); //direcitons  
  digitalWrite(DIR2, LOW);
  analogWrite(PWM1, 255);   //speed scale speed here with input from pi
  analogWrite(PWM2, 127);
  
  //Fade(PWM1, 5);
  //Fade(PWM2, 20); 
  delay(100);
}


//ma'fuck funcitons//////////////////////////////////////////////////////////////////////////////

void receiveData(int byteCount) {   //read data
  //int i = 0;
  while (Wire.available()) {
  /*  number[i] = Wire.read();
  //  i++;
  }
  number[i] = '\0';
  Serial.print(number);
  */
  number = Wire.read();
  if (number == 1){
   if (state == 0){
    digitalWrite(13, HIGH); // set the LED on
    state = 1;
   } else{
    digitalWrite(13, LOW); // set the LED off
    state = 0;
   }
  }
  Serial.println(number);
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



