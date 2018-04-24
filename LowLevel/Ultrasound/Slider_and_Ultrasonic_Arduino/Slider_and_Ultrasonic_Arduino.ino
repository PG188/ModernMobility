/*
This software reads the value from each of the ultrasonic sensors and manual slider.
Every 10ms these values are sent back to the Raspberry Pi for processing 
*/

#include <Arduino.h>
#include <Filters.h>

#define numOfUS 4
#define numOfSlider 2

//Defines pins numbers for each sensor
//Can be changed to whatever after wiring
const int trigPin[numOfUS] = {10,12,14,16};
const int echoPin[numOfUS] = {11,13,15,17};

// defines US variables
long duration[numOfUS] = {0,0,0,0}; //Holds propogation time of ultrasonic signal in microseconds
int distance_cm[numOfUS] = {0};

//Defines pins for sliders
const int leftSliderPin = 3;
const int rightSliderPin = 4;

//Stores analog slider readings
int leftSliderVal = 0;
int rightSliderVal = 0;

byte ValArray[(numOfUS + numOfSlider)*2] = {0};

//Serial variables
int serial_start_flag = 0;
int readByte;

//Low Pass Filter variables
float filterFrequency_lslider = 2;
float filterFrequency_rslider = 2;
FilterOnePole lowPassFilter_lslider(LOWPASS, filterFrequency_lslider);
FilterOnePole lowPassFilter_rslider(LOWPASS, filterFrequency_rslider);

void setup() {
    Serial.begin(115200); // Starts the serial communication at 57600 baud (this is fast enough)
    /*for (int i=0; i<numOfUS; i++) {
        pinMode(trigPin[i], OUTPUT); // Sets the trigPin as an Output
        pinMode(echoPin[i], INPUT); // Sets the echoPin as an Input   
    }*/
    // create a one pole (RC) lowpass filter
    pinMode(leftSliderPin, INPUT);
    pinMode(rightSliderPin, INPUT);
}
void loop() {
    //First we read the analog values for the sliders
    lowPassFilter_lslider.input(analogRead(leftSliderPin));
    lowPassFilter_rslider.input(analogRead(rightSliderPin));
  
    leftSliderVal = (int) lowPassFilter_lslider.output();
    //Serial.print("Left Slider Value = ");Serial.print(leftSliderVal);
    rightSliderVal = (int) lowPassFilter_rslider.output();
    //Serial.print("\tRight Slider Value = ");Serial.println(rightSliderVal);
    //leftSliderVal = 512;
    //rightSliderVal = 513;
    distance_cm[0] = 100;
    distance_cm[1] = 200;
    distance_cm[2] = 300;
    distance_cm[3] = 400;
    
    ValArray[0] = byte(leftSliderVal & 0x00FF);
    ValArray[1] = byte((leftSliderVal >> 8) & 0x00FF);
    ValArray[2] = byte(rightSliderVal & 0x00FF);
    ValArray[3] = byte((rightSliderVal >> 8) & 0x00FF);
    for (int j=0; j<numOfUS; j++){
      ValArray[4+j*2] = byte(distance_cm[j] & 0x00FF);
      ValArray[5+j*2] = byte((distance_cm[j] >> 8) & 0x00FF);
    }
    /*
     * Any slider filter/amplification if necessary.
     */
     //Serial.println(Serial.available());
     if (Serial.available()) {
       readByte = Serial.read();
       if (readByte == 'A') {
          serial_start_flag = 1; //Start serial communication
       } else if (readByte == 'Z') {
          serial_start_flag = 0;
          Serial.write('Z'); //Acknowledge stop command
       }
     }
     if (serial_start_flag) {
      Serial.write(ValArray,12);
     }
     delay(30);
}
