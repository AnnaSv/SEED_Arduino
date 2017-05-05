/*
  UVM SEED Team #22
  Beta Air
  Quadcopter Arduino Sketch - Simplified

  This program reads three linear accelerations from a 3-axis accelerometer
  and three angular velocities from a 3-axis gyroscope located on a
  6 Degree of Freedom Internal Measurement Unxit shield (6 DOF IMU).
  The shield has an embedded Xbee socket which is used to transmit
  these values to an Arduino located on the simulator platform.

  FreeSixIMU Library is available here:
  https://github.com/simondlevy/FreeSixIMU2.git

  Written By: Marco White, Tariye Peter, Anna Svagzdys
  Last Edited: 4/18/17
*/

// Load the 6 DOF IMU shield libraries
#include <FreeSixIMU.h>
#include <FIMU_ADXL345.h>
#include <FIMU_ITG3200.h>

// Load Wire library
#include <Wire.h>

// load timer interrupt library
#include <TimerOne.h>

// Define the FreeSixIMU object
FreeSixIMU sixDOF = FreeSixIMU();

// Array to hold the raw accelerometer and gyroscope data
int rawSixDof[6];

// Array to hold some rough calibration values
int calib[6];
int count = 0;

// desired data rate in Hz
const int data_rate = 100;

int gyro_sum = 0;
int gyro_x = 0;
int samples = 0;
char buf[2];

// Setup loop
void setup()
{
  // Initialize serial and wire
  Serial.begin(9600);
  Serial.flush();
  Wire.begin();

  // intialize timer interrupt
  Timer1.initialize(1000000);
  Timer1.attachInterrupt(send_data, 1000000 / data_rate);

  // Initialize cailbration values
  for (int i = 0; i < 6; i++) {
    calib[i] = 0;
  }
  // Set LED pin 13 as an output and write it low
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);

  // Initialize the IMU
  sixDOF.init();

  count++;
  if (count >= 10 && count < 20) {
    if (count == 10) {
      for (int i = 0; i < 6; i++) {
        calib[i] = rawSixDof[i];
      }
    }
    else {
      for (int i = 0; i < 6; i++) {
        calib[i] = (calib[i] + rawSixDof[i]) / 2;
      }
    }
  }

}

// Main loop
void loop() {

  // Read the raw sensor values
  sixDOF.getRawValues(rawSixDof);

  // keep running average of gyro
  gyro_x = rawSixDof[4] - calib[4];

}

void send_data() {

   // send data
  //Serial.println(gyro_x);
  buf[0] = gyro_x & 0xff;
  buf[1] = (gyro_x >> 8) & 0xff;
  Serial.write(buf, 2);

}

