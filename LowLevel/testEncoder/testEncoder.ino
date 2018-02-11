/* Encoder Library - TwoKnobs Example
 * http://www.pjrc.com/teensy/td_libs_Encoder.html
 *
 * This example code is in the public domain.
 */
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#define SAMPLE_DELAY (10)   //  this gets 1 reading per second.
                              //  adjust the delay to fit your needs
#define PULSES_PER_TURN (512)  //  32 state changes per turn on 1 line, 
unsigned int lastTime; 
float rpm;                    // speed in turns/minute, doesn't have to be floating point.

long lastPosition = 0;
// Change these pin numbers to the pins connected to your encoder.
//   Best Performance: both pins have interrupt capability
//   Good Performance: only the first pin has interrupt capability
//   Low Performance:  neither pin has interrupt capability
Encoder knobLeft(3, 4);
//   avoid using pins with LEDs attached

void setup() {
  Serial.begin(9600);
  Serial.println("TwoKnobs Encoder Test:");
}

long positionLeft  = -999;

void loop() {
  long newLeft;
  newLeft = knobLeft.read();
  /*if (newLeft != positionLeft ) {
    Serial.print("Left = ");
    Serial.print(newLeft);
    Serial.println();
    positionLeft = newLeft;
  }*/
  positionLeft = newLeft;
    if ((unsigned int)millis() - lastTime >= SAMPLE_DELAY)  
      {
           rpm = ((positionLeft-lastPosition) * (60000.f / ((unsigned int)millis() - lastTime))) / PULSES_PER_TURN;
           Serial.print("RPM = "); Serial.print(rpm);Serial.print(" positionLeft = ");Serial.print(positionLeft);Serial.print(" lastPosition = ");Serial.println(lastPosition);
           lastTime = (unsigned int)millis();
           lastPosition = positionLeft;
           positionLeft = 0;
      }
  //}
}
