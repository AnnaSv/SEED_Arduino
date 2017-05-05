/*
UVM SEED Team #22
Beta Air
Pilot Interface Arduino Sketch

This program reads the throttle and handlebar potentiometers,
receives roll and pitch data over USB, then writes values 
to a hacked RC controller.

Written By: 

Marco White (SEED 2015-2016)
Anna Svagzdys (SEED 2016-2017)
Last Edited: 3/28/2017
*/

#include <SPI.h>

// Pin Definitions
#define POT1_SHDN 7
#define POT1_SS 10 
#define POT2_SHDN 8
#define POT2_SS 9 // rudder, throttle
#define POTS_SDI 11
#define POTS_SDO 12
#define POTS_CLK 13
#define THROTTLE_POT A3 
#define HANDLEBAR_POT A2 
#define INTERRUPT 3

// Define program constants

// Sensor minimum and maximum values for map funtion
#define THROTTLE_POT_MIN 165 
#define THROTTLE_POT_MAX 760
#define HANDLEBAR_POT_MIN 3 
#define HANDLEBAR_POT_MAX 344 
#define ROLL_CV_MIN -220 
#define ROLL_CV_MAX 220 
#define PITCH_CV_MIN -120 
#define PITCH_CV_MAX 120 

// Joystick minimum and maximum resistance values - DO NOT CHANGE
#define PITCH_MIN 10
#define PITCH_MAX 240 //220
#define ROLL_MIN 30
#define ROLL_MAX 240
#define THROTTLE_MIN 40
#define THROTTLE_MAX 220
#define YAW_MIN 20
#define YAW_MAX 240

// Digital potentiometer addresses
#define TCON_WRITE_ADDR B01000000
#define BOTH_WIPERS_ON B11111111
#define WIPER0_ADDR B00000000 // throttle
#define WIPER1_ADDR B00010000 // yaw

// Initialize global variables
int rollCV, pitchCV, throttlePot, handlebarPot;
int pitchVal, rollVal, throttleVal, yawVal;
char buf[4];
volatile char msg = 'y';
unsigned long timer = 0;

// Setup
void setup() {
  
  // Initialize serial monitor
  Serial.begin(9600);

  // Initialize interrupt pin to set neutral position
  pinMode(INTERRUPT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT), alert, FALLING);
  
  // Initialize digital potentiometers
  pinMode(POT1_SHDN, OUTPUT);
  pinMode(POT1_SS, OUTPUT);
  pinMode(POT2_SHDN, OUTPUT);
  pinMode(POT2_SS, OUTPUT);
  
  // Pulse the shutdown pins on both pots
  digitalWrite(POT1_SHDN, LOW);
  digitalWrite(POT2_SHDN, LOW);
  delay(10);
  digitalWrite(POT1_SHDN, HIGH);
  digitalWrite(POT2_SHDN, HIGH);
  
  // Initialize SPI (communication interface for digital pots)
  SPI.begin();
  
  // Start with both digital pots de-selected
  digitalWrite(POT1_SS, HIGH);
  digitalWrite(POT2_SS, HIGH);

  // Initialize timer
  timer = millis();

}

// Infinite loop
void loop() {

  // check if data is available from webcam
  if (Serial.available() > 0) {
    
    // read roll and yaw data bytes
    Serial.readBytes(buf,4);
    rollCV = word(buf[1],buf[0]);
    pitchCV = word(buf[3],buf[2]);

    /* Commment out except for debugging, it jams the comms.
    Serial.print("Lean roll: ");
    Serial.print(rollCV);
    Serial.print(" / Lean pitch: ");
    Serial.println(pitchCV);
    */
  }

  // Send ready msg back to laptop
  if ( ( millis() - timer ) > 100 ) {
    Serial.write(msg);
    timer = millis();
  }
  
  
  // Get analog potentiometer values
  throttlePot = analogRead(THROTTLE_POT);
  handlebarPot = analogRead(HANDLEBAR_POT);
  
  // Print sensor values
  /*
  Serial.print("   Roll: ");
  Serial.print(rollCV, DEC);
  Serial.print("   Pitch: ");
  Serial.print(pitchCV, DEC);
  Serial.print("   Yaw: ");
  Serial.print(handlebarPot, DEC);
  Serial.print("   Throttle: ");
  Serial.print(throttlePot, DEC);
  */
  
  // Map sensor values to corresponding joystick resistances (on the hacked RC controller)
  pitchVal = constrain(map(pitchCV, PITCH_CV_MIN, PITCH_CV_MAX, PITCH_MIN, PITCH_MAX), PITCH_MIN, PITCH_MAX);
  rollVal = constrain(map(rollCV, ROLL_CV_MIN, ROLL_CV_MAX, ROLL_MIN, ROLL_MAX), ROLL_MIN, ROLL_MAX);
  throttleVal = constrain(map(throttlePot, THROTTLE_POT_MIN, THROTTLE_POT_MAX, THROTTLE_MIN, THROTTLE_MAX), THROTTLE_MIN, THROTTLE_MAX);
  yawVal = constrain(map(handlebarPot, HANDLEBAR_POT_MIN, HANDLEBAR_POT_MAX, YAW_MIN, YAW_MAX), YAW_MIN, YAW_MAX);
  
  // Print digital pot values
  /*
  Serial.print("   Pitch: ");
  Serial.print(pitchVal, DEC);
  Serial.print("   Roll: ");
  Serial.print(rollVal, DEC);
  Serial.print("   Throttle: ");
  Serial.print(throttleVal, DEC);
  Serial.print("   Yaw: ");
  Serial.println(yawVal, DEC);
  */
  
  // Turn on both wipers on each digital pot
  digitalPotWrite(POT1_SS, TCON_WRITE_ADDR, BOTH_WIPERS_ON);
  digitalPotWrite(POT2_SS, TCON_WRITE_ADDR, BOTH_WIPERS_ON);
  
  // Write the mapped resistance values to the digital potentiometers
  digitalPotWrite(POT1_SS, WIPER0_ADDR, rollVal);  
  digitalPotWrite(POT1_SS, WIPER1_ADDR, pitchVal); 
  digitalPotWrite(POT2_SS, WIPER0_ADDR, throttleVal); // ok
  digitalPotWrite(POT2_SS, WIPER1_ADDR, yawVal); // ok
  
}

// Method to write to the digital potentiometers
void digitalPotWrite(int pin, int addr, int val) {
  // Set the slave select pin low to select the chip
  digitalWrite(pin, LOW);
  // Send the address and value via SPI
  SPI.transfer(addr);
  SPI.transfer(val);
  // De-select the chip
  digitalWrite(pin, HIGH);
}

// method to send msg when interrupt is triggered
void alert() {
 static unsigned long last_interrupt_time = 0;
 unsigned long interrupt_time = millis();
 // If interrupts come faster than 200ms, assume it's a bounce and ignore
 if (interrupt_time - last_interrupt_time > 50)
 {
   msg = 'y';
 }
 last_interrupt_time = interrupt_time;
}

