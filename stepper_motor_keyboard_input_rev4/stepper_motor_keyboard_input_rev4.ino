
/* Stepper Motor Keyboard Speed Input Test
   Anna Svagzdys
   4/17/2017

   This code reads in speed and direction commands over USB
   and adjusts the direction and PFM speed of the motor accordingly.
   This is accomplished by changing the PWM duty cycle.

   Note that there is no provision to keep track of motor angle here.

   Timer library documentation + download available here:
   http://playground.arduino.cc/Code/Timer1

*/

#include "TimerThree.h"

char buf[4];
int pps = 0;
int dir = 1;

void setup() {

  // Set up PWM -> PFM pin
  Timer3.initialize(1000000); // initialize timer3
  Timer3.pwm(5, 0); // setup pwm on pin 5, 0% duty cycle to start with
  
  // Begin Serial comms
  Serial.begin(9600);
  Serial.flush();

  // Set up Pin 22 & 24 to use as direction pin;
  DDRA = DDRA | B00000101;

}

void loop() {

  // check if new instructions are available from computer
  if (Serial.available() > 0) {

    // read step and direction data
    Serial.readBytes(buf, 4);
    pps = word(buf[1], buf[0]);
    dir = word(buf[3], buf[2]);

    // compute new pulse period
    double ppsDouble = (double) pps; // this is stupid
    double cycleMicros = 1000000 / ppsDouble;

    if (pps > 0) {

      // set direction
      if (dir > 0) {
        PORTA = B00000100;
      } else {
        PORTA = B00000001;
      }

      // set period
      Timer3.pwm(5, 20, cycleMicros); // pin, dutyCycle, period
   
    } else { //turn that shit off!

      Timer3.pwm(5, 0, cycleMicros); // pin, dutyCycle, period
  
    }
  }
}

