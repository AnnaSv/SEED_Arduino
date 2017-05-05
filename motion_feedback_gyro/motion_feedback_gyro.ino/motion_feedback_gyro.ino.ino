
/* Stepper Motor Control Code
   Anna Svagzdys
   4/4/2017

   This code reads in roll-axis gyro data from an XBee radio
   module and coverts the angular acceleration to a lean angle
   command for the stepper motors.

   We will set up the XBee to send data at a known frequency,
   so the approximate delay between serial events is used to
   compute the PFM frequency sent to the motors.

   For our 200 CPR optical encoder and 29:1 gearmotor, each pulse
   should move the motor shaft 0.06 degrees, with max pulse rate of 6 kHz.

   Max pulse rate tested is 5kHz, which drove motor at around 4RPS.

   TO DO:
     Testing
     EOT limits - count must have magnitude < 1500 (test this)
     Init stage - in setup(), move motors until both are at theta = 0
     Lazy washout filter - if theta != 0 and accel is below some threshhold,
   move motors slowly back toward theta = 0
     Zero-position update - reset count to 0 when theta = 0

*/

#include "TimerOne.h"

const int pfmPin = 9; // step pin
// pins 5 and 7 used for port manip

const double dataFreq = 100; // in Hz, how often we can expect accel data from XBee
const double motionFreq = 10; // in Hz, how often motion commands are computed
const double maxPFMFreq = 5000;
const double minPFMFreq = 0;
const double minGyro = 0;
const double maxGyro = 2000;
const double max_return_pfm = 200;

const double degs_per_step = 0.06 / 4;
const int accel_threshold = 10;

char buf[2];
int gyro_x = 0;
int last_gyro_x = 0;
int gyro_delta = 0;
int gyro_bias = 0;
int adjusted_gyro = 0;

double degs = 0;
int dir = 1;
int pfmVal = 0;

void setup() {

  // Set up PWM -> PFM pin
  Timer1.initialize(1000000); // initialize timer3
  Timer1.pwm(pfmPin, 0); // setup pwm on pin 5, 0% duty cycle to start with

  // Set up Pins 7 & 5 to use as direction pin;
  DDRD = DDRD | B10100000;

  // Begin serial comms
  Serial.begin(9600);
  Serial.flush();

  // attach timer interrupt for motor comms
  Timer1.attachInterrupt(set_motor_speed, (int) 1000000 / motionFreq);

  // do init: turn until sensors report 0deg, then pause for 1 sec

}

void loop() {

  // poll accelerometer when data is ready and take moving avg
  if (Serial.available() > 0) {

    //store old gyro data
    last_gyro_x = gyro_x;

    // get new gyro data
    Serial.readBytes(buf, 2);
    gyro_x = word(buf[1], buf[0]);
    Serial.print("Gyro: ");
    Serial.println(gyro_x);

    gyro_delta = gyro_x - last_gyro_x;

    if (abs(gyro_delta) < accel_threshold) {

      // if gyro is small, save any non-zero accel for gyro bias offset
      gyro_bias = last_gyro_x;

      // if system isnt at neutral, go there slowly
      if (abs(degs) > 0) {

        // get return freq,
        pfmVal = (degs * motionFreq) / degs_per_step;

        //set to max return freq if too high
        if (pfmVal > max_return_pfm) {
          pfmVal = max_return_pfm;
        }

        // set motion direction
        if (degs < 0) { // move forward
          dir = 1;
        } else if (degs > 0) { // move backward
          dir = 0;
        }

      } else {
        // system is at neutral, do nothing
        pfmVal = 0;
      }

    } else {

      // if accel is large, map gyro value to a PWM frequency
      adjusted_gyro = gyro_x - gyro_bias;
      pfmVal = constrain(map(adjusted_gyro, minGyro, maxGyro, minPFMFreq, maxPFMFreq), minPFMFreq, maxPFMFreq);

      // set motion direction
      if (adjusted_gyro > 0) { // pos accel
        dir = 1;
      } else if (adjusted_gyro < 0) { // neg accel
        dir = 0;
      }
    }
  }
}

// interrupt method to determine PFM and send to motor
void set_motor_speed() {

  // set direction
  if (dir == 1) { // move forward
    PORTD = B00000101;
  } else { // move backward
    PORTD = B00000000;
  }

  // set pfm freq
  if (pfmVal > 0) {
    Timer1.pwm(pfmPin, 20, (int)(1000000 / pfmVal));
  } else {
    Timer1.pwm(pfmPin, 0, (int)(1000000 / pfmVal));
  }
  Serial.print("PFM: ");
  Serial.println(pfmVal);
}



