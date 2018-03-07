#include <Arduino.h>

#define L_PIN_1_IN 0
#define L_PIN_2_IN 1
#define R_PIN_1_IN 1
#define R_PIN_2_IN 1

#define L_PIN_OUT 1
#define R_PIN_OUT 1

#define THRESHOLD 201

    //Indices for encoder arrays below
const int PIN_1_IN = 0,   //Index for left/right pin 1 analog input
          PIN_2_IN = 1,   //Index for left/right pin 2 analog input
          TICK = 2,       //Index for left/right encoder count
          PIN_1 = 3,      //Index for left/right pin 1 state (High/Low)
          PIN_2 = 4,      //Index for left/right pin 2 state (High/Low)
          PIN_1_LAST = 5, //Index for left/right pin 1 previous state (High/Low)
          PIN_2_LAST = 6, //Index for left/right pin 1 previous state (High/Low)
          PIN_OUT = 7;    //Index for left/right digital output pin
 
int l_encoder[8] = { L_PIN_1_IN, L_PIN_2_IN, 0, LOW, LOW, LOW, LOW, L_PIN_OUT }, 
    r_encoder[8] = { R_PIN_1_IN, R_PIN_2_IN, 0, LOW, LOW, LOW, LOW, R_PIN_OUT };

int outputSens;
int maxRange = 0; int minRange = 1000;

void setup() {
    Serial.begin(115200);
    pinMode(L_PIN_1_IN, INPUT);
    pinMode(L_PIN_2_IN, INPUT);
}

void loop() {
    //Serial.println(analogRead(L_PIN_1_IN));
    /*if (analogRead(L_PIN_1_IN) >= 258) {
      outputSens = 1;
    }
    else {
      outputSens = 0;
    }
    Serial.println(outputSens);*/
    Serial.print("Pin1 = ");Serial.println(analogRead(L_PIN_1_IN));//Serial.print("  Pin2 = ");Serial.println(analogRead(L_PIN_2_IN));
    /*outputSens = analogRead(L_PIN_2_IN);
    if (outputSens > maxRange) {
      maxRange = outputSens;
    }
    if (outputSens < minRange) {
      minRange = outputSens;
    }*/
    //Serial.println(digitalRead(L_PIN_1_IN));
    //Serial.print("max = "); Serial.print(maxRange); Serial.print("  min = "); Serial.println(minRange);
    //readEncoder(l_encoder);
    
  /*
    readEncoder(l_encoder);
    Serial.println(l_encoder[TICK], DEC);
    */
}

int a2d(int analog_input){
    return ((analog_input >= THRESHOLD) ? HIGH : LOW);
}

void readEncoder(int encoder[]){
    encoder[PIN_1] = a2d(analogRead(encoder[PIN_1_IN]));
    encoder[PIN_2] = a2d(analogRead(encoder[PIN_2_IN]));
    //Serial.print("Pin1 = ");Serial.print(encoder[PIN_1]);Serial.print("  Pin2 = ");Serial.println(encoder[PIN_2]);
    bool pin1_rising = (encoder[PIN_1] == HIGH) && (encoder[PIN_1_LAST] == LOW),
          pin2_rising = (encoder[PIN_2] == HIGH) && (encoder[PIN_2_LAST] == LOW);

    //If pin 1 is rising add to count if pin 2 is HIGH, otherwise subtract
    if(pin1_rising){ encoder[TICK] += ((encoder[PIN_2] == HIGH) ? 1 : -1); }
    encoder[PIN_1_LAST] = encoder[PIN_1];
    
    //If pin 2 is rising add to count if pin 1 is LOW, otherwise subtract
    if(pin2_rising){ encoder[TICK] += ((encoder[PIN_1] == LOW) ? 1 : -1);}
    encoder[PIN_2_LAST] = encoder[PIN_2];

    Serial.println(encoder[TICK]);
}

void writeEncoderTick(int encoder[]){
    digitalWrite(encoder[PIN_OUT], encoder[TICK]);
}

