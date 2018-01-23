/*
This software reads the value from each of the ultrasonic sensors and manual slider.
Every 10ms these values are sent back to the Raspberry Pi for processing 
*/


/*
Issues: The stop reset is not working. Could stem from the bytes potentially ariving after the 
first block of code with the Serial.available check. How can we make it so this cannot happen?
*/

#include <Arduino.h>

int byteRead = 0;
int encoder = 100;
int STOP_FLAG = 126;
int START_FLAG = 127;
float motorVelCmd = 0;
byte sendEncoder = 0;

void setup() {
    Serial.begin(115200); // Starts the serial communication at 57600 baud (this is fast enough)
    delay(2000);
}
void loop() {
    // Receive new motor command or stop/start
    if (Serial.available()) {
      byteRead = Serial.read();
      if (byteRead == START_FLAG) { //Start command
        sendEncoder = 1;
      } else if (byteRead == STOP_FLAG) { //Stop command
        sendEncoder = 0;
      } else { //Velocity command
        /*If byteRead is negative, its MSB will be 1 because 2s compliment. 
        * However, because arduino is dumb, it reads in bytes as 16-bit integers,
        * So, when you try to read the value, its 8 MSB are zero, making
        * Arduino think it is a positive number. We must set these bits to 1
        * (sign extension) to get it to recognize byteRead as negative.
        */
        if (byteRead & 0x0080) { //Is byteRead supposed to be negative?
          byteRead = byteRead | 0xFF00; //Sign extends byteRead to 16 bits
        }
        motorVelCmd = float(byteRead)/100; //Converts cm/s value to m/s
      }  
    }
    
    //DO PID
    
    //UPDATE ENCODER  
    
    //Send updated encoder value
    if (sendEncoder == 1) {
      Serial.write(byte(encoder & 0x00FF)); 
      Serial.write(byte((encoder >> 8) & 0x00FF));
    }
    delay(30);
}
