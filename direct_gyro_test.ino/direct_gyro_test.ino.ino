
/* Stepper Motor Control Code
   Anna Svagzdys
   4/4/2017

   This code reads in roll-axis gyro data and coverts the
   angular acceleration to a lean angle command for the
   motors.

   TODO:
     Testing
     EOT limits - count must have magnitude < 1500 (test this)
     Implement optical switches as zero-position update
     Init stage - in setup(), move motors until both are at theta = 0
     Zero-position update - reset count to 0 when theta = 0

*/

// Load timer interrupt library
#include <TimerOne.h>

#include <Wire.h>

// Load the 6 DOF IMU shield libraries
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

FreeSixIMU imu = FreeSixIMU();

const int pfm_pin = 9; // step pin
// pins 0 and 2 used for port manip

const double degs_per_step = 0.06 / 4;
const double accel_threshold = 20;
const double neutral_threshold = 2;
const double data_rate = 100; // in Hz, how often we can expect accel data from XBee
const double update_rate = 10; // in Hz, how often motion commands are computed
const double max_pfm = 1200; // conservative...
const double min_pfm = 0;
const double min_gyro = accel_threshold;
const double max_gyro = 2000;
const double max_return_pfm = 40;

int gyro_x = 0;
int last_gyro_x = 0;
int gyro_delta = 0;
int gyro_bias = 0;
int adjusted_gyro = 0;

int raw_data[6];
int calib[6];
int count = 0;

int gyro_sum = 0;
int samples = 0;

double degs = 0;
int dir = 1;
int pfm = 0;

void setup() {

    // Initialize serial and wire
  Serial.begin(9600);
  Serial.flush();

  Wire.begin();
  
  // Set up PWM -> PFM pin
  Timer1.initialize(1000000); // initialize timer3
  Timer1.pwm(pfm_pin, 0); // setup pwm on pin 5, 0% duty cycle to start with

  // attach timer interrupt for motor comms
  Timer1.attachInterrupt(set_motor_speed, (int) 1000000 / update_rate);

  // Set up Pins 7 & 5 to use as direction pin;
  DDRD = DDRD | B00000101;
  
  // Initialize cailbration values
  for (int i = 0; i < 6; i++) {
    calib[i] = 0;
  }

  // Initialize the IMU
  imu.init();

  // perform calibration
  count++;
  if (count >= 10 && count < 20) {
    if (count == 10) {
      for (int i = 0; i < 6; i++) {
        calib[i] = raw_data[i];
      }
    }
    else {
      for (int i = 0; i < 6; i++) {
        calib[i] = (calib[i] + raw_data[i]) / 2;
      }
    }
  }

  // do init: turn until sensors report 0deg, then pause for 1 sec

}

void loop() {

  Serial.println(degs);
  //Serial.println(pfm);

  // save last gyro val
  last_gyro_x = gyro_x;

  // Read the raw sensor values
  imu.getRawValues(raw_data);

  // compute average
  gyro_sum += raw_data[4] - calib[4];
  samples++;
  gyro_x = gyro_sum / samples;
  //Serial.println(gyro_x);

  gyro_delta = gyro_x - last_gyro_x;
  //Serial.println(gyro_delta);
  
  if (abs(gyro_delta) < accel_threshold) {

    // if gyro is small, save any non-zero accel for gyro bias offset
    //gyro_bias = last_gyro_x;
    //Serial.println(gyro_bias);

    // if system isnt at neutral, go there slowly
    if (abs(degs) > neutral_threshold) {

      // get return freq,
      pfm = abs((degs * update_rate) / degs_per_step);

      //set to max return freq if too high
      if (pfm > max_return_pfm) {
        pfm = max_return_pfm;
      }

      // set motion direction
      if (degs < -neutral_threshold) { // move forward
        Serial.println("Returning to Neutral...");
        dir = 1;
        degs = degs + (pfm * degs_per_step * (1/update_rate));
      } else if (degs > neutral_threshold) { // move backward
        Serial.println("Returning to Neutral...");
        dir = 0;
        degs = degs - (pfm * degs_per_step * (1/update_rate));
      }

    } else {
      // system is at neutral, do nothing
      Serial.println("At neutral");
      pfm = 0;
    }

  } else {

    Serial.println("Tracking motion...");
    
    // if accel is large, map gyro value to a PWM frequency
    adjusted_gyro = gyro_x - gyro_bias;
    
    pfm = constrain(map(abs(adjusted_gyro), min_gyro, max_gyro, min_pfm, max_pfm), min_pfm, max_pfm);

    // set motion direction
    if (adjusted_gyro > 0) { // pos accel
      dir = 1;
      degs = degs + (pfm * degs_per_step * (1/update_rate));
    } else if (adjusted_gyro < 0) { // neg accel
      dir = 0;
      degs = degs - (pfm * degs_per_step * (1/update_rate));
    }
  }
}

// interrupt method to determine PFM and send to motor
void set_motor_speed() {

   /* NOTE: Printing to serial from this interrupt does not work!
   *  If you want to know if the code is working, use a scope.
   */

  // reset running avg params
  gyro_sum = 0;
  samples = 0;

  // set direction
  if (dir == 1) { // move forward
    PORTD = B00000101;
  } else { // move backward
    PORTD = B00000000;
  }

  // set pfm freq
  if (pfm > 0) {
    Timer1.pwm(pfm_pin, 20, (int)(1000000 / pfm));
  } else {
    Timer1.pwm(pfm_pin, 0, (int)(1000000 / pfm));
  }  
}



