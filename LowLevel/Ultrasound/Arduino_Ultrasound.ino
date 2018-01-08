/*
This software reads the value from each of the ultrasonic sensors and 
sends it back to the Raspberry Pi. The strategy is to send out the pulse
and detect the response of one sensor, then move on to the next. This will
reduce interference and seems to be the best way to meet the timing requirements
of the sensors.The time needed to cycle through one sensor is very small so this 
shouldn't be an issue
*/

#include <Arduino.h>

#define numOfUS 4

//Defines pins numbers for each sensor
//Can be changed to whatever after wiring
const int trigPin[numOfUS] = {10,12,14,16};
const int echoPin[numOfUS] = {11,13,15,17};

// defines variables
long duration = 0; //Holds propogation time of ultrasonic signal in microseconds
int distance_cm[numOfUS] = {0};

void setup() {
    Serial.begin(19200); // Starts the serial communication at 19200 baud (this is fast enough)
    for (int i=0; i<numOfUS; i++) {
        pinMode(trigPin[i], OUTPUT); // Sets the trigPin as an Output
        pinMode(echoPin[i], INPUT); // Sets the echoPin as an Input   
    }
}
void loop() {
    for (int i=0; i<numOfUS; i++) {
        // Clears the trigPin
        digitalWrite(trigPin[i], LOW);
        delayMicroseconds(2);
        // Sets the trigPin on HIGH state for 10 micro seconds
        digitalWrite(trigPin[i], HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin[i], LOW);
        // Reads the echoPin, returns the sound wave travel time in microseconds
        duration = pulseIn(echoPin[i], HIGH);
        // Calculates the distance in centimeters.
        // The distance will be converted into meters on the Pi
        distance_cm[i]= duration*0.034/2;
    }
    //Sends the distances over serial. Integers should be two bytes
    for (int i=0; i<numOfUS; i++) {
        Serial.write(byte(distance_cm[i] & 0x00FF));
        Serial.write(byte((distance_cm[i] >> 8) & 0x00FF));
    }
    delay(10); //Loop operates every 10 milliseconds
}