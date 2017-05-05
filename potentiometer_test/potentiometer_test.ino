#define THROTTLE A3 
#define RUDDER A2  

void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int throttleValue = analogRead(THROTTLE);
  // print out the value you read:
  Serial.print("Throttle value: ");
  Serial.println(throttleValue);
  delay(1);
  
    // read the input on analog pin 0:
  int rudderValue = analogRead(RUDDER);
  // print out the value you read:
  Serial.print("Rudder value: ");
  Serial.println(rudderValue);
  delay(500);        // delay in between reads for stability
}
