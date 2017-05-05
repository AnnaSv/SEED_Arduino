
/* Stepper Motor Angle Input Test
   Anna Svagzdys
   4/4/2017

   This code reads in a degree value (relative to starting position)
   over serial and moves the motor shaft to that position.

*/

#include "TimerThree.h"

const int pfmPin = 3;

char buf[2];
const double pfmFreq = 1000; 
const double pulseDegs = 0.062/4;// why is this so far off!?

int new_angle = 0;
int last_angle = 0;
int delta_angle = 0;

double duration = 0.0;

void setup() {

  // Set up PWM -> PFM pin
  Timer3.initialize(1000000); // initialize timer3
  Timer3.pwm(pfmPin, 0); // setup pwm on pin 5, 0% duty cycle to start with

  // Set up Pin 22 & 24 to use as direction pin;
  DDRA = DDRA | B00000101;

  Serial.begin(9600);
  Serial.flush();

}

void loop() {

  // poll accelerometer when data is ready
  if (Serial.available() > 0) {
    
    Serial.readBytes(buf, 2);
    new_angle = word(buf[1], buf[0]);
    delta_angle = new_angle - last_angle;
    duration = abs(delta_angle/(pfmFreq*pulseDegs));

    if (delta_angle > 0) {
      PORTA = B00000101;
    } else {
      PORTA = B00000000;
    }

    Serial.print("Moving motor to ");
    Serial.println(new_angle);
    Timer3.pwm(pfmPin, 20, (int)(1000000/pfmFreq)); 
    delay(1000*duration);
    Timer3.pwm(pfmPin, 0, (int)(1000000/pfmFreq)); 
    Serial.println("Motion complete");

    last_angle = new_angle;
    
  }

}

