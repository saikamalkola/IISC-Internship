//these pins can not be changed 2/3 are special pins
int encoderPin1 = 2;
int encoderPin2 = 3;
 
volatile int lastEncoded = 0;
volatile long encoderValue = 0;
 
long lastencoderValue = 0;
 
int lastMSB = 0;
int lastLSB = 0;
 
void setup() {
  Serial.begin (9600);
 
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);
 
  //digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
 // digitalWrite(encoderPin2, HIGH); //turn pullup resistor on
 
  //call updateEncoder() when any high/low changed seen
  //on interrupt 0 (pin 2), or interrupt 1 (pin 3) 
  attachInterrupt(0, updateEncoder, CHANGE); 
 // attachInterrupt(1, updateEncoder, CHANGE);
 
}
 
void loop(){ 
  //Do stuff here
 
  Serial.println(encoderValue);
  delay(10); //just here to slow down the output, and show it will work  even during a delay
  Serial.flush();
}
 
 
void updateEncoder(){
  encoderValue++;
}
