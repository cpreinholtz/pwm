/*
  Analog input, analog output, serial output

  Reads an analog input pin, maps the result to a range from 0 to 255 and uses
  the result to set the pulse width modulation (PWM) of an output pin.
  Also prints the results to the Serial Monitor.

  The circuit:
  - potentiometer connected to analog pin 0.
    Center pin of the potentiometer goes to the analog pin.
    side pins of the potentiometer go to +5V and ground
  - LED connected from digital pin 9 to ground

  created 29 Dec. 2008
  modified 9 Apr 2012
  by Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInOutSerial
*/

// These constants won't change. They're used to give names to the pins used:
//const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to
const int analogOutPin = 9; // Analog output pin that the LED is attached to

const int upPin = 6;
const int dnPin = 4;//  normally closed buttons, connected to gnd

int sensorValue = 0;        // value read from the pot

const int analogOutPin2 = 3; // Analog output pin that the LED is attached to


const int upPin2 = 7;
const int dnPin2 = 8;//  normally closed buttons, connected to gnd

int sensorValue2 = 0;        // value read from the pot


//int outputValue = 0;        // value output to the PWM (analog out)

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);

  pinMode(upPin, INPUT_PULLUP);
  pinMode(dnPin, INPUT_PULLUP);
  pinMode(upPin2, INPUT_PULLUP);
  pinMode(dnPin2, INPUT_PULLUP);

  TCCR1B = TCCR1B & 0b11111000 | 0x01;
}

void loop() {

  if (digitalRead(dnPin)){
    sensorValue = sensorValue -1;
    if (sensorValue < 0){ sensorValue = 0;}   
  } else if (digitalRead(upPin)){
    sensorValue = sensorValue +1;
    if (sensorValue > 255){ sensorValue = 255;}    
  }
  
  if (digitalRead(dnPin2)){
    sensorValue2 = sensorValue2 -1;
    if (sensorValue2 < 0){ sensorValue2 = 0;}   
  } else if (digitalRead(upPin2)){
    sensorValue2 = sensorValue2 +1;
    if (sensorValue2 > 255){ sensorValue2 = 255;}    
  }

  
  // map it to the range of the analog out:
  //outputValue = map(sensorValue, 0, 1023, 0, 255);
  // change the analog out value:
  analogWrite(analogOutPin, sensorValue);
  
  analogWrite(analogOutPin2, sensorValue2);
  
  Serial.print("sensor = ");
  Serial.println(sensorValue);

  
  // wait 2 milliseconds before the next loop for the analog-to-digital
  // converter to settle after the last reading:
  delay(10);
}
