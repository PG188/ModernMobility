#include <Arduino.h>

enum something {a, b, c};

#define L_ENCODER_PIN1 1
#define L_ENCODER_PIN2 1
#define R_ENCODER_PIN1 1
#define R_ENCODER_PIN2 1

#define L_ENCODER_COUNT 1
#define R_ENCODER_COUNT 1

#define THRESHOLD 511

//"pin" variables represetnt the HIGH/LOW state of the pin
int l_encoder_count, l_pin1, l_pin2, l_pin1_last, l_pin2_last,
    r_encoder_count, r_pin1, r_pin2, r_pin1_last, r_pin2_last;

bool l_pin1_rising, l_pin2_rising, r_pin1_rising, r_pin2_rising;

void setup() {
    Serial.begin(115200);
    
    l_encoder_count = 0;
    r_encoder_count = 0;
    
    l_pin1_last = LOW;
    l_pin2_last = LOW;
    r_pin1_last = LOW;
    r_pin2_last = LOW;
}

void loop() {
    readLeftEncoder();
    readRightEncoder();
}

int a2d(int analogInput){
    return ((analogInput >= THRESHOLD) ? HIGH : LOW);
}

void readLeftEncoder(){
    l_pin1 = a2d(analogRead(L_ENCODER_PIN1));
    l_pin2 = a2d(analogRead(L_ENCODER_PIN2));
    l_pin1_rising = (l_pin1 == HIGH) && (l_pin1_last == LOW);
    l_pin2_rising = (l_pin2 == HIGH) && (l_pin2_last == LOW);

    //If pin 1 is rising add to count if pin 2 is HIGH, otherwise subtract
    if(l_pin1_rising){ l_encoder_count += ((l_pin2 == HIGH) ? 1 : -1); }
    l_pin1_last = l_pin1;

    //If pin 2 is rising add to count if pin 1 is LOW, otherwise subtract
    if(l_pin2_rising){ l_encoder_count += ((l_pin1 == LOW) ? 1 : -1); }
    l_pin2_last = l_pin2;
}

void readRightEncoder(){
    r_pin1 = a2d(analogRead(R_ENCODER_PIN1));
    r_pin2 = a2d(analogRead(R_ENCODER_PIN2));
    r_pin1_rising = (r_pin1 == HIGH) && (r_pin1_last == LOW);
    r_pin2_rising = (r_pin2 == HIGH) && (r_pin2_last == LOW);

    //If pin 1 is rising add to count if pin 2 is HIGH, otherwise subtract
    if(r_pin1_rising){ r_encoder_count += ((r_pin2 == HIGH) ? 1 : -1); }
    r_pin1_last = r_pin1;

    //If pin 2 is rising add to count if pin 1 is LOW, otherwise subtract
    if(r_pin2_rising){ r_encoder_count += ((r_pin1 == LOW) ? 1 : -1); }
    r_pin2_last = r_pin2;
}



void writeLeftEncoder(){
    
}



